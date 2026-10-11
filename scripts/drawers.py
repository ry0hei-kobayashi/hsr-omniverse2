"""段違い棚 (wrc_stair_like_drawer) の trofast を PrismaticJoint で「本物の引き出し」にする (launch_isaacsim.py から分離)。"""
import os

import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


# ============================================================
# 引き出し (段違い棚 wrc_stair_like_drawer の trofast) を「本物の引き出し」にする
# ============================================================
# world ファイルの trofast_* は段違い棚 (wrc_stair_like_drawer) に「置いてあるだけ」の
# 自由剛体で、そのままではロボットが取っ手を掴んで引くことができない。理由は 2 つ:
#
#   1. trofast_knob/model.usd の /body に ArticulationRootAPI が付いている。
#      これが付いた物体は PhysX が別アーティキュレーションとして扱い、ロボット
#      (これもアーティキュレーション) と衝突しなくなる = 指がすり抜ける。
#      YCB は drop_object() の [obj-fix] で除去しているが、world の <include> 経由で
#      読まれる trofast はその経路を通らないため付いたままだった。
#
#   2. 棚のレール間隔は 0.280 m なのに箱の上縁は 0.300 m あり、左右 10 mm ずつ
#      最初から食い込んでいる。実物の「縁をレールに引っ掛けて吊る」構造をそのまま
#      USD にしたためで、剛体シミュレーションでは初期めり込みになる。PhysX が
#      これを押し出そうとして箱が弾かれたり楔状に噛んで沈み込んだりする。
#
# 対策として、棚と箱の間に PrismaticJoint (直動ジョイント) を張り、レールの代わりに
# ジョイントで運動を拘束する。ジョイントで繋いだ 2 体は collisionEnabled=False により
# 当たり判定が切れるので、上記 2. のめり込みも同時に解消する。結果として
# 「手前にだけスライドし、引き切ると止まり、抜け落ちない」本物の引き出しになる。
# 段違い棚の無い world (他タスク) では何もしない (no-op)。
DRAWER_FRAME_PATH = '/wrc_stair_like_drawer'
# 引き出しを何 m 引き出せるか (ジョイントの上限)。箱の奥行きは 0.4235 m。
DRAWER_PULL_LIMIT = float(os.environ.get('DRAWER_PULL_LIMIT', '') or '0.30')
# 引き出しの粘性抵抗 [N/(m/s)]。大きいほど重い引き心地になる。0 で無抵抗。
# 指の把持力 (hsr.py の FINGER_MAX_FORCE) で引ける範囲に収めること。
DRAWER_DAMPING = float(os.environ.get('DRAWER_DAMPING', '') or '15.0')
# 粘性抵抗が出せる力の上限 [N]。引き出しを止める力ではなく減衰の頭打ち。
DRAWER_DAMPING_MAX_FORCE = float(
    os.environ.get('DRAWER_DAMPING_MAX_FORCE', '') or '200.0')
# 0 にすると引き出し化を丸ごと止めて従来の「置いてあるだけの箱」に戻せる。
DRAWER_JOINTS_ENABLED = os.environ.get('DRAWER_JOINTS', '1') != '0'

# 引き出しにした箱の /body パス。[recol] の対象外にするために覚えておく
# ([recol] は setRigidBody で剛体を作り直すため、張ったジョイントが壊れる)。
drawer_body_paths = []


def strip_articulation_root(prim):
    """prim 以下の ArticulationRootAPI を外して「ただの剛体」に戻す。

    drop_object() の [obj-fix] と同じ処理。付いたままだとロボットの指がすり抜ける。
    """
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            print('[drawer] removed ArticulationRootAPI from %s' % p.GetPath(),
                  flush=True)
        if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
            p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)


def setup_drawers(model_names):
    """段違い棚と trofast_* を PrismaticJoint で繋いで引き出しにする。

    :param model_names: world から spawn したモデル名の一覧 (trofast* を探す)
    """
    _stage = omni.usd.get_context().get_stage()
    frame = _stage.GetPrimAtPath(DRAWER_FRAME_PATH)
    frame_link = _stage.GetPrimAtPath(DRAWER_FRAME_PATH + '/link')
    if not frame.IsValid() or not frame_link.IsValid():
        # この world に段違い棚が無い (別アリーナ) なら何もしない。
        return

    # 棚もアーティキュレーションを外して素の剛体に戻したうえで kinematic にする。
    # <static> により world への FixedJoint は張られているが、それだけだと剛体
    # (mass 1kg) のままなので引き出しを引く反力で棚が揺れる。kinematic にすれば
    # 完全に不動のアンカーになり、ジョイントの相手として安定する。
    strip_articulation_root(frame)
    UsdPhysics.RigidBodyAPI.Apply(frame_link).CreateKinematicEnabledAttr(True)

    frame_inv = UsdGeom.Xformable(frame_link).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).GetInverse()

    for name in model_names:
        if not name.startswith('trofast'):
            continue
        root = _stage.GetPrimAtPath('/' + name)
        body_path = '/' + name + '/body'
        body = _stage.GetPrimAtPath(body_path)
        if not root.IsValid() or not body.IsValid():
            print('[drawer] skip %s (no /body prim)' % name, flush=True)
            continue

        # 1. 指がすり抜ける原因の ArticulationRootAPI を外す。
        strip_articulation_root(root)

        # 2. 棚に対する箱の現在の相対姿勢を測り、そこをジョイント原点 (= 閉じた状態)
        #    にする。world ファイルに書かれた姿勢がそのまま「閉」になるので、
        #    座標をコードに焼き込まずに済む。
        body_l2w = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        rel = body_l2w * frame_inv
        rel_pos = rel.ExtractTranslation()
        rel_rot = rel.GetOrthonormalized().ExtractRotationQuat()

        # 3. 直動ジョイント。軸は棚ローカルの X = ワールド +Y = 引き出しの手前方向。
        #    (棚は yaw=90° で置かれ、前面の桟が world の +Y 側にある)
        joint = UsdPhysics.PrismaticJoint.Define(
            _stage, Sdf.Path('/' + name + '/drawer_joint'))
        joint.CreateBody0Rel().SetTargets([DRAWER_FRAME_PATH + '/link'])
        joint.CreateBody1Rel().SetTargets([body_path])
        joint.CreateAxisAttr('X')
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel_pos))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(rel_rot))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        # 0 = 閉じきり、DRAWER_PULL_LIMIT = 引ききり。ここで機械的に止まる。
        joint.CreateLowerLimitAttr(0.0)
        joint.CreateUpperLimitAttr(DRAWER_PULL_LIMIT)
        # 棚と箱の当たり判定を切る (既定でも False だが、レールとの初期めり込みを
        # 確実に無効化したいので明示する)。動きはジョイントが拘束するので、
        # 当たり判定を切っても箱が棚をすり抜けて落ちることはない。
        joint.CreateCollisionEnabledAttr(False)

        # 4. 粘性抵抗。これが無いと引いた勢いでストッパーに激突して跳ね返る。
        #    stiffness=0 なので「戻ろうとする力」は働かず、引いた位置で止まる。
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(DRAWER_DAMPING)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateMaxForceAttr(DRAWER_DAMPING_MAX_FORCE)

        # 5. 掴んで静止すると眠ってしまい、次に押しても反応しなくなるのを防ぐ
        #    ([recol] が同じことをしているが、引き出しは [recol] 対象外にするため)。
        _rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
        _rb.CreateDisableGravityAttr(False)
        _rb.CreateSleepThresholdAttr(0.0)

        drawer_body_paths.append(body_path)
        print('[drawer] %s -> prismatic joint (0..%.2f m, damping=%.1f)'
              % (name, DRAWER_PULL_LIMIT, DRAWER_DAMPING), flush=True)
