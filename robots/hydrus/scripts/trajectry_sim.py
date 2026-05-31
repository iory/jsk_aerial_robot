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


URDF_PATH = "/home/tokunaga/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/hydrus/robots/quad/tilt_0deg_ce_15inch_202604/robot.urdf"

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
def attach_world_floating_base(robot):
    """world と root_link の間に 6 DoF FloatingJoint を挿入し 0 で固定."""
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
    # 6 DoF を 0 に固定 (= world 上で root が原点 + 単位回転)
    fjoint.joint_angle(np.zeros(6))
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=240)
    parser.add_argument('--radius', type=float, default=0.125)
    parser.add_argument('--dt', type=float, default=0.02)
    parser.add_argument('--resolution', type=int, nargs=2, default=(960, 720))
    parser.add_argument('--update-interval', type=float, default=0.02)
    parser.add_argument('--trail-points', type=int, default=80)
    parser.add_argument('--ik-stop', type=int, default=20)
    parser.add_argument('--fix-joint', type=str, default='joint2',
                        choices=['joint1', 'joint2', 'joint3'],
                        help='固定する関節 (残り 2 つを IK 変数にする)')
    parser.add_argument('--fix-angle', type=float, default=0.6,
                        help='--fix-joint で指定した関節の固定値 [rad]')
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # URDF 読み込み
    # ------------------------------------------------------------------
    print('Loading URDF:', URDF_PATH)
    robot = RobotModelFromURDF(urdf_file=URDF_PATH)
    print('  root_link =', robot.root_link.name)
    print('  joints    =', [j.name for j in robot.joint_list])  
    
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
    world_link, fjoint = attach_world_floating_base(robot)
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
    for j in range(args.trail_points):
        phi = 2.0 * np.pi * j / args.trail_points
        wp = p0 + np.array([args.radius * np.cos(phi),
                            args.radius * np.sin(phi),
                            0.0])
        mark = Sphere(radius=0.008, pos=wp)
        mark.set_color([60, 120, 220, 255])   # 青系
        viewer.add(mark)

    # 現在ターゲットを示す大きめの軸 (毎フレーム動く)
    target_axis = skrobot.model.Axis(
        axis_radius=0.015, axis_length=0.18, pos=p0.copy())
    viewer.add(target_axis)
    # world 軸 (= root, 固定)
    viewer.add(skrobot.model.Axis(
        axis_radius=0.008, axis_length=0.20, pos=(0, 0, 0)))

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
    k = 0
    errors = []
    log_every = max(1, args.steps // 12)

    def step_once(k):
        theta = 2.0 * np.pi * (k % args.steps) / args.steps
        target_xyz = p0 + np.array([args.radius * np.cos(theta),
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
        return target_xyz, err, ok

    if viewer is None:
        # ヘッドレス: 1 周だけ回して数値出力
        for k in range(args.steps):
            _, err, _ = step_once(k)
            errors.append(err)
    else:
        # インタラクティブ: 閉じられるまでずっと回す
        # IKを解くとrootのjointが勝手に更新
        while viewer.is_active:
            target_xyz, err, _ = step_once(k)
            errors.append(err)
            # target を可視化マーカーに反映 (これも破壊更新)
            target_axis.newcoords(Coordinates(pos=target_xyz))
            viewer.redraw()
            time.sleep(args.dt)
            k += 1

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