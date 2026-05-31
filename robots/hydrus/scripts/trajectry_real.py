#!/usr/bin/env python

import argparse
import os
import time

import numpy as np

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.model import Link
from skrobot.model.joint import FloatingJoint
from skrobot.models.urdf import RobotModelFromURDF

import rospy
import tf
from tf.transformations import euler_from_quaternion

from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Vector3Stamped

URDF_PATH = "/home/tokunaga/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/hydrus/robots/quad/tilt_0deg_ce_15inch_202604/robot.urdf"

# xyz,royをROSのtfから取得
def get_xyz_rpy(listener, parent_frame, child_frame):
    trans, quat = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
    xyz = np.array(trans, dtype=float)
    rpy = np.array(euler_from_quaternion(quat), dtype=float)
    return xyz, rpy

# ============================================================================
# Q1) floating base な関節 (6 DoF) の与え方
# ============================================================================
#
# 与え方は 2 通り:
# (A) 手動取り付け: world (仮想 link) → FloatingJoint → robot.root_link
# (B) `inverse_kinematics(use_base='6dof')` で IK 内部に一時挿入させる
#
# 今回は "root を world に固定したい" 用途なので (A) で挿入して 0 で固定する.
# ----------------------------------------------------------------------------
def attach_world_floating_base(robot, fixed_root_xyz, root_rpy0):
    """world と root_link の間に 6 DoF FloatingJoint を挿入."""
    """ xyz は node開始時のROS値で固定し、rpy は外から更新する. """
    world_link = Link(name='world')
    fjoint = FloatingJoint(
        parent_link=world_link,
        child_link=robot.root_link,
        name='world_to_root',
    )
    # parent/child を上書き接続
    robot.root_link._parent_link = world_link
    world_link.add_child_link(robot.root_link)
    robot.root_link.joint = fjoint
    #  xyz は node開始時のROS値で固定．rpy は初期値で挿入し，以降は外から更新する．
    root_xyz_rpy = np.r_[fixed_root_xyz, root_rpy0]
    fjoint.joint_angle(root_xyz_rpy)
    return world_link, fjoint


def solve_one_step(robot, leg5, target_coords, link_list, stop=20):
    """IK を 1 ステップ解いて (success, leg5_pos) を返す.
    (Q4) root 固定 + joint2 0.6 固定下での IK 呼び出し
    (Q5) inverse_kinematics は破壊更新なので呼び出し後 robot.* に反映される
    stop はこの問題では 10 以降 精度ほぼ頭打ち (実測). デフォルト 20 で
    per-call ~2.4 ms / 50 Hz refresh と滑らかな整合.
    """
    result = robot.inverse_kinematics(
        target_coords,
        link_list=link_list,
        move_target=leg5,
        position_mask='xy',    # xy のみ追従 (平面リンク機構)
        rotation_mask=False,   # 姿勢は free
        stop=stop,
        revert_if_fail=False,
    )
    ok = result is not False and result is not None
    leg5_pos = leg5.worldpos()
    return ok, leg5_pos

def report_step(k, target_xyz, leg5_pos, robot, ok):
    print(
        f'  step={k:>3d}  target_xy=({target_xyz[0]:+.3f},'
        f'{target_xyz[1]:+.3f})  '
        f'leg5_xy=({leg5_pos[0]:+.3f},{leg5_pos[1]:+.3f})  '
        f'err={float(np.linalg.norm(leg5_pos[:2] - target_xyz[:2])):.4f}  '
        f'ok={ok}  '
        f'joints=({robot.joint1.joint_angle():+.3f},'
        f'{robot.joint2.joint_angle():+.3f},'
        f'{robot.joint3.joint_angle():+.3f})')

def pub_command_values(nav_pub, joint_pub, cog_cmd, q_cmd, yaw_cmd):
    now = rospy.Time.now()

    # COG + yaw command
    nav_msg = FlightNav()
    nav_msg.header.stamp = now
    nav_msg.header.frame_id = 'world'
    nav_msg.control_frame = FlightNav.WORLD_FRAME
    nav_msg.target = FlightNav.COG
    nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
    nav_msg.target_pos_x = float(cog_cmd[0])
    nav_msg.target_pos_y = float(cog_cmd[1])
    nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
    nav_msg.target_pos_z = float(cog_cmd[2])
    nav_msg.yaw_nav_mode = FlightNav.POS_MODE
    nav_msg.target_yaw = float(yaw_cmd)
    nav_pub.publish(nav_msg)

    # Joint command
    joint_msg = JointState()
    joint_msg.header.stamp = now
    joint_msg.name = ['joint1', 'joint2', 'joint3']
    joint_msg.position = [
        float(q_cmd[0]),
        float(q_cmd[1]),
        float(q_cmd[2]),
    ]
    joint_pub.publish(joint_msg)

# joint変化量を制限する関数
def pub_limit_joint(q_cmd, q_target, max_step):
    diff = q_target - q_cmd
    diff_limited = np.clip(diff, -max_step, max_step)
    return q_cmd + diff_limited

# COG変化量を制限する関数
def pub_limit_nav(pos_cmd, pos_target, max_step):
    diff = pos_target - pos_cmd
    norm = np.linalg.norm(diff)
    if norm < max_step or norm < 1e-6:
        return pos_target
    return pos_cmd + diff/norm * max_step

# joint_states を subscribe して最新値を保持するクラス
class JointStateReader:
    def __init__(self):
        # 初期値はとりあえず中心値を入れておく
        self.q = {'joint1': 1.0, 'joint2': 0.6, 'joint3': 1.0}
        self.sub = rospy.Subscriber('/hydrus/joint_states', JointState, self._callback, queue_size=1)

    def _callback(self, msg):
        q_dict = dict(zip(msg.name, msg.position))
        for j in ['joint1', 'joint2', 'joint3']:
            if j in q_dict:
                self.q[j] = q_dict[j]

    def get_actual_q(self):
        """最新の [q1, q2, q3] を numpy 配列で返す"""
        return np.array([self.q['joint1'], self.q['joint2'], self.q['joint3']], dtype=float)

# debug確認用のVector3Stampedメッセージ作成関数
def make_vec3_msg(vec, frame_id='world'):
    msg = Vector3Stamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.vector.x = float(vec[0])
    msg.vector.y = float(vec[1])
    msg.vector.z = float(vec[2])
    return msg

# debug用のベクトルをpublishする関数
def pub_debug_vectors(debug_pubs, leg5_xyz, target_xyz, fixed_root_xyz, root_xyz):
    leg5_error = leg5_xyz - target_xyz
    root_error = root_xyz - fixed_root_xyz

    debug_pubs['leg5'].publish(make_vec3_msg(leg5_xyz))
    debug_pubs['target_leg5'].publish(make_vec3_msg(target_xyz))
    debug_pubs['leg5_error'].publish(make_vec3_msg(leg5_error))
    debug_pubs['target_root'].publish(make_vec3_msg(fixed_root_xyz))
    debug_pubs['root'].publish(make_vec3_msg(root_xyz))
    debug_pubs['root_error'].publish(make_vec3_msg(root_error))

# debug情報をpublishする関数
def publish_debug(listener, debug_pubs, fixed_root_xyz, target_xyz):
    leg5_xyz, _ = get_xyz_rpy(
        listener,
        parent_frame='world',
        child_frame='hydrus/leg5'
    )

    root_xyz, _ = get_xyz_rpy(
        listener,
        parent_frame='world',
        child_frame='hydrus/root'
    )

    pub_debug_vectors(
        debug_pubs=debug_pubs,
        leg5_xyz=leg5_xyz,
        target_xyz=target_xyz,
        fixed_root_xyz=fixed_root_xyz,
        root_xyz=root_xyz
    )

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=1000)
    parser.add_argument('--radius', type=float, default=0.10)
    parser.add_argument('--dt', type=float, default=0.02)
    parser.add_argument('--resolution', type=int, nargs=2, default=(960, 720))
    parser.add_argument('--update-interval', type=float, default=0.002)
    parser.add_argument('--trail-points', type=int, default=80)
    parser.add_argument('--ik-stop', type=int, default=20)
    parser.add_argument('--control-hz',type=float, default=50.0)
    parser.add_argument('--fix-joint', type=str, default='joint2',
                        choices=['joint1', 'joint2', 'joint3'],
                        help='固定する関節 (残り 2 つを IK 変数にする)')
    parser.add_argument('--fix-angle', type=float, default=0.6,
                        help='--fix-joint で指定した関節の固定値 [rad]')
    parser.add_argument('--q1_center', type=float, default=1.0,
                        help='joint1 の初期値 (中心値) [rad]')
    parser.add_argument('--q3_center', type=float, default=1.0,
                        help='joint3 の初期値 (中心値) [rad]')
    parser.add_argument('--max-cog-step', type=float, default=0.0001,
                        help='COG コマンドの最大変化量 [m]')
    parser.add_argument('--max-q-step', type=float, default=0.001,
                        help='joint コマンドの最大変化量 [rad]')
    args = parser.parse_args()
    
    # ----------------------------------------------------------------
    # ros登録
    # ----------------------------------------------------------------
    rospy.init_node('trajectory_real', anonymous=True)
    listener = tf.TransformListener()

    nav_pub = rospy.Publisher('/hydrus/uav/nav', FlightNav, queue_size=1)
    joint_pub = rospy.Publisher('/hydrus/joints_ctrl', JointState, queue_size=1)
    joint_reader = JointStateReader()
    rospy.sleep(1.0)  # tf の listener がちゃんと動き出すまで待つ
    rate = rospy.Rate(args.control_hz) # ROSのRateでループの周期を制御 (例: 50 Hz -> 0.02s)
    debug_pubs = {
        'leg5': rospy.Publisher('/debug/leg5', Vector3Stamped, queue_size=1),
        'target_leg5': rospy.Publisher('/debug/target_leg5', Vector3Stamped, queue_size=1),
        'leg5_error': rospy.Publisher('/debug/leg5_error', Vector3Stamped, queue_size=1),
        'target_root': rospy.Publisher('/debug/target_root', Vector3Stamped, queue_size=1),
        'root': rospy.Publisher('/debug/root', Vector3Stamped, queue_size=1),
        'root_error': rospy.Publisher('/debug/root_error', Vector3Stamped, queue_size=1),
    }

    # ------------------------------------------------------------------
    # URDF 読み込み
    # ------------------------------------------------------------------
    print('Loading URDF:', URDF_PATH)
    robot = RobotModelFromURDF(urdf_file=URDF_PATH)
    print('  root_link =', robot.root_link.name)
    print('  joints    =', [j.name for j in robot.joint_list])  
    
    # ------------------------------------------------------------------
    # rosから現在のrootの位置姿勢取得
    # ------------------------------------------------------------------
    fixed_root_xyz, root_rpy0 = get_xyz_rpy(listener, parent_frame='world', child_frame='hydrus/root')
    print('fixed root xyz =', fixed_root_xyz)
    print('initial root rpy =', root_rpy0)

    # ------------------------------------------------------------------
    # joint1, joint3 の可動範囲を設定
    # ------------------------------------------------------------------
    robot.joint1.min_angle=0.3
    robot.joint1.max_angle=1.47
    robot.joint3.min_angle=0.3
    robot.joint3.max_angle=1.47

    # ------------------------------------------------------------------
    # (Q1) Floating base を挿入
    # ------------------------------------------------------------------
    world_link, fjoint = attach_world_floating_base(robot, fixed_root_xyz, root_rpy0)
    print('floating-base joint:', fjoint.type, ', dof =', fjoint.joint_dof,
          ', angles =', fjoint.joint_angle())

    # ------------------------------------------------------------------
    # (Q4 の前半) 1 つの関節 (joint2) を args.fix_angle rad で固定
    # ------------------------------------------------------------------
    fixed_joint = getattr(robot, args.fix_joint)
    fixed_joint.joint_angle(args.fix_angle)
    print('{} fixed at {} rad'.format(args.fix_joint, fixed_joint.joint_angle()))

    # ------------------------------------------------------------------
    # (Q2) move_target の与え方
    # ------------------------------------------------------------------
    # `move_target` = 追従させたい EE の座標系 (Coordinates).
    # leg5 は link4 に fixed joint で剛体接続されたフレームなので Link
    # オブジェクトを直接渡せばよい:
    # joint1, joint3 の以下の初期値を中心とした円軌道となる．
    # copy()は其の時点での値の独立したコピーとなるため，必要．
    robot.joint1.joint_angle(1.0)
    robot.joint3.joint_angle(1.0)
    leg5 = robot.leg5
    p0 = leg5.worldpos().copy()
    print('leg5 worldpos (initial) =', p0)

    # ------------------------------------------------------------------
    # (Q4 の本体) IK 用の link_list
    # ------------------------------------------------------------------
    # `link_list` は「可動 joint を子に持つ link」を並べる仕様.
    # root と joint1 を固定したいので含めない:
    #   joint2 (link2 → link3) → link3
    #   joint3 (link3 → link4) → link4
    JOINT_TO_CHILD_LINK = {
        'joint1': robot.link2,
        'joint2': robot.link3,
        'joint3': robot.link4,
    }
    link_list = [link for jname, link in JOINT_TO_CHILD_LINK.items()
                 if jname != args.fix_joint]
    print('IK link_list = {}  → 変数 joints = {}'.format(
        [l.name for l in link_list],
        [l.joint.name for l in link_list]))
    print(f'reference: circle r={args.radius} m around {p0}')

    # ------------------------------------------------------------------
    # Viewer セットアップ (PyrenderViewer)
    # ------------------------------------------------------------------
    viewer = None
    target_axis = None
    # update_interval が大きいと viewer の描画が遅くてカクついて見える.
    # デフォルト 1.0s (= 1Hz) は明らかに遅いので 0.02s (50Hz) 程度に.
    viewer = skrobot.viewers.PyrenderViewer(
        resolution=tuple(args.resolution),
        update_interval=args.update_interval)
    viewer.add(robot)

    # ★ 参照円軌道をあらかじめ小さい球の列で描いておく.
    # IK ループが追従すべき経路が一目で分かる.
    from skrobot.model.primitives import Sphere
    trail_spheres = []
    for j in range(args.trail_points):
        phi = 2.0 * np.pi * j / args.trail_points
        wp = p0 + np.array([args.radius * np.cos(phi),
                            args.radius * np.sin(phi),
                            0.0])
        mark = Sphere(radius=0.008, pos=wp)
        mark.set_color([60, 120, 220, 255])   # 青系
        viewer.add(mark)
        trail_spheres.append(mark)

    # 現在ターゲットを示す大きめの軸 (毎フレーム動く)
    target_axis = skrobot.model.Axis(
        axis_radius=0.015, axis_length=0.18, pos=p0.copy())
    viewer.add(target_axis)
    # root 軸 (毎フレーム動く)
    root_axis = skrobot.model.Axis(
        axis_radius=0.008, axis_length=0.20, pos=fixed_root_xyz)
    viewer.add(root_axis)

    viewer.show()
    print()
    print('==> PyrenderViewer is running '
            '(update_interval={:.3f}s).'.format(args.update_interval))
    print('    Blue spheres = 参照円軌道, '
            '大きい軸 = 現在ターゲット, ロボット leg5 が追従.')
    print('    Close the window (or press [q]) to stop the IK loop.')
    print()

    # ------------------------------------------------------------------
    # メインループ: while viewer.is_active:
    #
    # - viewer が閉じられるまで参照円軌道を回し続ける
    # - 各イテレーションで target を更新 → IK を解く → 軸を動かす → 再描画
    # ------------------------------------------------------------------
    START_ERR_TOL = 0.01
    RADIUS_ERR_TOL = 0.01
    STABLE_COUNT_REQUIRED = 500
    stable_count = 0
    mode = 'SET_MODE'
    center_world = None
    k = 0
    errors = []
    log_every = max(1, args.steps // 12)
    q_cmd = np.array([robot.joint1.joint_angle(), robot.joint2.joint_angle(), robot.joint3.joint_angle()])
    cog_cmd = robot.centroid().copy()


    def step_once(k, center_xyz):
        # rosから現在のjoint角度取得
        q_actual = joint_reader.get_actual_q()
        
        # 現在のjoint角度をrobotに反映
        robot.joint1.joint_angle(q_actual[0])
        robot.joint2.joint_angle(q_actual[1])
        robot.joint3.joint_angle(q_actual[2])

        # rosから現在のroot姿勢取得
        _, root_rpy = get_xyz_rpy(listener, parent_frame='world', child_frame='hydrus/root')

        # rootの姿勢を更新
        q_root = np.r_[fixed_root_xyz, root_rpy]
        
        # rosから現在のroot姿勢取得
        # root_xyz, root_rpy = get_xyz_rpy(listener, parent_frame='world', child_frame='hydrus/root')

        # rootの姿勢を更新
        # q_root = np.r_[root_xyz, root_rpy]
        fjoint.joint_angle(q_root)

        # 参照円軌道上の target_xyz を計算
        theta = 2.0 * np.pi * (k % args.steps) / args.steps
        target_xyz = center_xyz + np.array([args.radius * np.cos(theta),
                                            args.radius * np.sin(theta),
                                            0.0])
        # (Q3) target_coords を組み立て
        target_coords = Coordinates(pos=target_xyz)

        # (Q4) IK を解く (= robot の joint を破壊更新)
        ok, leg5_pos = solve_one_step(
            robot, leg5, target_coords, link_list, stop=args.ik_stop)

        err = float(np.linalg.norm(leg5_pos[:2] - target_xyz[:2]))
        if k % log_every == 0 or not ok:
            report_step(k, target_xyz, leg5_pos, robot, ok)
        return target_xyz, err, root_rpy, ok

    if viewer is None:
        # ヘッドレス: 1 周だけ回して数値出力
        for k in range(args.steps):
            _, err, _, _ = step_once(k, center_xyz=p0)
            errors.append(err)
    else:
        # インタラクティブ: 閉じられるまでずっと回す
        # IKを解くとrootのjointが勝手に更新
        while viewer.is_active:
            if mode == 'SET_MODE':
                # rosから現在のroot姿勢取得
                root_xyz, root_rpy = get_xyz_rpy(listener, parent_frame='world', child_frame='hydrus/root')
                fjoint.joint_angle(np.r_[root_xyz, root_rpy])
                # 円中心なるときのjointを設定
                robot.joint1.joint_angle(args.q1_center)
                robot.joint2.joint_angle(args.fix_angle)
                robot.joint3.joint_angle(args.q3_center)
                # 円中心なるときのee位置を計算
                center_candidate = leg5.worldpos().copy()
                # 円軌道上の開始点を計算してそこにtargetを配置
                start_target_candidate = center_candidate + np.array([args.radius, 0.0, 0.0])
                target_coords = Coordinates(pos=start_target_candidate)
                # IKを解いてleg5の位置を計算して,pub
                _, leg5_pos = solve_one_step(robot, leg5, target_coords, link_list, stop=args.ik_stop)
                q_target = np.array([robot.joint1.joint_angle(), robot.joint2.joint_angle(), robot.joint3.joint_angle()])
                cog_target = robot.centroid().copy()
                _, cog_rpy = get_xyz_rpy(listener, parent_frame='world', child_frame='hydrus/fc')
                cog_yaw = cog_rpy[2]
                q_cmd = np.array(q_cmd)  
                cog_cmd = np.array(cog_cmd)  
                q_cmd = pub_limit_joint(q_cmd, q_target, args.max_q_step)
                cog_cmd = pub_limit_nav(cog_cmd, cog_target, args.max_cog_step)
                pub_command_values(nav_pub, joint_pub, cog_cmd, q_cmd, cog_yaw)
                try:
                    actual_leg5_xyz, _ = get_xyz_rpy(listener, parent_frame='world', child_frame='hydrus/leg5')
                    start_err = float(np.linalg.norm(actual_leg5_xyz[:2] - start_target_candidate[:2]))
                    radius_err = float(np.linalg.norm(actual_leg5_xyz[:2] - center_candidate[:2]) - args.radius)
                    
                    if start_err < START_ERR_TOL and abs(radius_err) < RADIUS_ERR_TOL:
                        stable_count += 1
                    else:
                        stable_count = 0
                    
                    rospy.loginfo_throttle(
                        1.0,
                        'SET_MODE: start_err={:.4f}, radius_err={:.4f}, stable={}/{}'.format(start_err, radius_err, stable_count, STABLE_COUNT_REQUIRED)
                    )
                    # --- 収束したらTEST_MODEへ移行 ---
                    if stable_count >= STABLE_COUNT_REQUIRED:
                        center_world = center_candidate.copy()
                        # 参照軌道作成
                        for j, mark in enumerate(trail_spheres):
                            phi = 2.0 * np.pi * j / args.trail_points
                            new_wp = center_world + np.array([args.radius * np.cos(phi),
                                                              args.radius * np.sin(phi),
                                                              0.0])
                            mark.newcoords(Coordinates(pos=new_wp))
                        # 初期値を保存してTEST_MODEへ
                        q_cmd = q_cmd.copy()
                        cog_cmd = cog_cmd.copy()
                        cog_yaw = cog_yaw
                        fixed_root_xyz = root_xyz.copy()
                        mode = 'TEST_MODE'
                        k = 0
                        errors = []
                        rospy.loginfo('>> switch to TEST_MODE: center_world = {}'.format(center_world))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
                # 50Hz制御
                rate.sleep()
            # ============================================================
            # 収束したらTEST_MODEへ
            # ============================================================
            elif mode == 'TEST_MODE':
                target_xyz, err, root_rpy, _ = step_once(k, center_xyz=center_world)
                errors.append(err)
                # 得られた値をpub
                q_cmd = pub_limit_joint(q_cmd, np.array([robot.joint1.joint_angle(), robot.joint2.joint_angle(), robot.joint3.joint_angle()]), args.max_q_step)
                cog_cmd = pub_limit_nav(cog_cmd, robot.centroid().copy(), args.max_cog_step)
                _, cog_rpy = get_xyz_rpy(listener, parent_frame='world', child_frame='hydrus/fc')
                cog_yaw = cog_rpy[2]
                pub_command_values(nav_pub, joint_pub, cog_cmd, q_cmd, cog_yaw)
                # debug情報をpublish
                publish_debug(listener, debug_pubs, fixed_root_xyz, target_xyz)
                # target を可視化マーカーに反映 (これも破壊更新)
                target_axis.newcoords(Coordinates(pos=target_xyz))
                # root の姿勢も可視化マーカーに反映(rotについてはyaw,pitch,rollの順で入れる)
                root_axis.newcoords(Coordinates(pos=fixed_root_xyz, rot=[root_rpy[2], root_rpy[1], root_rpy[0]]))
                report_step(k, target_xyz, leg5.worldpos(), robot, True)
                viewer.redraw()
                rate.sleep()
                k += 1                
        viewer.redraw()
        rate.sleep()  # time.sleep(args.dt) の代わりに ROSのRateを使用
    # ------------------------------------------------------------------
    # まとめ
    # ------------------------------------------------------------------
    print('mean xy error = {:.4f} m, max = {:.4f} m  (over {} steps)'.format(
        float(np.mean(errors)), float(np.max(errors)), len(errors)))

    # ------------------------------------------------------------------
    # (Q5) 解いた後の angle_vector の確認
    # ------------------------------------------------------------------
    # `inverse_kinematics` は破壊更新. ループ末時点の関節角を確認:
    # - 単発取得:   robot.joint2.joint_angle()
    # - 個別設定:   robot.joint2.joint_angle(value)
    # - 一括ベクトル: robot.angle_vector() / robot.angle_vector(av)
    av = robot.angle_vector()
    print('current angle_vector =', np.round(av, 4))
    print('  (joint2 が 0.6 のまま固定されていることを確認)')

    # ------------------------------------------------------------------
    # (Q6) root 座標系から見た COG の取得
    # ------------------------------------------------------------------
    # robot.centroid() は world 座標系 COG.
    # root から見たい場合は world→root の変換で持ち込む.
    cog_world = robot.centroid()
    root_T_world = robot.root_link.copy_worldcoords()\
                                   .inverse_transformation()
    cog_root = root_T_world.transform_vector(cog_world)
    print('COG (world) =', cog_world)
    print('COG (root)  =', cog_root)
    print('total mass  =',
          robot._cached_mass_props['total_mass'], 'kg')
    # 今回 root は world に固定なので cog_world == cog_root.


if __name__ == '__main__':
    main()