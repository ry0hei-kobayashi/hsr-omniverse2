"""環境変数で調整する hsr.py のパラメータ群 (hsr.py から分離。値と意味はコメント参照)。"""
import os


# 案A(アタッチ把持)を使うか。既定は無効で、指と物体の物理接触(摩擦)だけで
# 掴む「正攻法」で動く。シミュレーションとして素直な挙動になり、
# 把持の失敗も物理的に起きるべくして起きる。
#
# 有効にすると物体が指に吸い付き、確実な保持と綺麗なリリースになる。
# 物理調整に手を取られたくないときや、把持より後段を試したいときに使う。
#   make up GRASP=1
_ATTACH_GRASP_ENABLED = os.environ.get('GRASP_ATTACH', '0') != '0'
# 真値odom: 台車の odom を Isaac 物理の真値(base_footprint 姿勢)で上書きし、計画・制御・TF を
# すべて真値で一致させる。車輪スリップや疎な環境でのレーザ破綻を避けられる(Simだから使える手)。
# 起動時に odometry_switcher を wheel_odom(=この真値) に向ける処理は hsr.launch.py 側。
# False にすると従来の車輪積分 odom に戻る。
_GT_ODOM = True
def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        print(f'[hsr] invalid {name}; using {default}', flush=True)
        return default
def _env_int(name, default):
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        print(f'[hsr] invalid {name}; using {default}', flush=True)
        return default
def _env_bool(name, default):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ('1', 'true', 'yes', 'on')
# Isaac の車輪 velocity drive は速度目標の急変で揺れやすいので、sim 起動時に
# 環境変数で調整できるようにする。
_BASE_DIRECT_DRIVE = _env_bool('BASE_DIRECT_DRIVE', True)
_BASE_TRAJ_P_GAIN = _env_float('BASE_TRAJ_P_GAIN', 2.0)
_BASE_TRAJ_D_GAIN = max(
    0.0, _env_float('BASE_TRAJ_D_GAIN', 0.5))
_BASE_GOAL_TOLERANCE = max(
    0.0, _env_float('BASE_GOAL_TOLERANCE', 0.010))
_BASE_CONTROL_DEADBAND = max(
    0.0, _env_float('BASE_CONTROL_DEADBAND', 0.003))
# 位置は平面ジョイントのサーボで固定されるので、到達判定は位置誤差で見る。
# 速度の許容値を 0.01 と厳しくすると、腕を動かした反動の微小な揺れだけで
# 「目標未達」となり whole_body 動作が失敗する (実測: 位置誤差 0.8mm でも失敗)。
_BASE_GOAL_VELOCITY_TOLERANCE = max(
    0.0, _env_float('BASE_GOAL_VELOCITY_TOLERANCE', 0.10))
_BASE_GOAL_SETTLE_DURATION = max(
    0.0, _env_float('BASE_GOAL_SETTLE_DURATION', 0.20))
_BASE_GOAL_TIMEOUT = max(
    0.0, _env_float('BASE_GOAL_TIMEOUT', 10.0))
_BASE_CMD_TAU = max(0.0, _env_float('BASE_CMD_TAU', 0.10))
# cmd_vel が途切れてから指令を無効にするまでの猶予。非ゼロ指令のまま送信が
# 途絶えると、この時間ぶん最後の指令で走り続ける。ゼロ指令を受けた場合は
# 速度値の判定で即座に停止するので、この値は関係しない。
_BASE_CMD_VEL_TIMEOUT = max(
    0.0, _env_float('BASE_CMD_VEL_TIMEOUT', 0.3))
_BASE_TRAJ_I_GAIN = max(
    0.0, _env_float('BASE_TRAJ_I_GAIN', 4.0))
_BASE_TRAJ_I_LINEAR_LIMIT = max(
    0.0, _env_float('BASE_TRAJ_I_LINEAR_LIMIT', 0.08))
_BASE_TRAJ_I_ANGULAR_LIMIT = max(
    0.0, _env_float('BASE_TRAJ_I_ANGULAR_LIMIT', 0.15))
_BASE_WHEEL_ACCEL_LIMIT = max(
    0.0, _env_float('BASE_WHEEL_ACCEL_LIMIT', 41.7))
_BASE_STEER_ACCEL_LIMIT = max(
    0.0, _env_float('BASE_STEER_ACCEL_LIMIT', 5.0))
_BASE_LINEAR_ACCEL_LIMIT = max(
    0.0, _env_float('BASE_LINEAR_ACCEL_LIMIT', 0.35))
_BASE_ANGULAR_ACCEL_LIMIT = max(
    0.0, _env_float('BASE_ANGULAR_ACCEL_LIMIT', 0.8))
_BASE_WHEEL_DRIVE_DAMPING = max(
    0.0, _env_float('BASE_WHEEL_DRIVE_DAMPING', 10.0))
# キャスター(受動輪)の転がり軸の粘性摩擦。0 だと軸受けが完全に無摩擦になり、
# 台車が動いたときに床の摩擦で回されたキャスターが、台車が止まった後も
# 慣性で永久に回り続ける (実測: 20秒間 10.06 rad/s から 1e-4 も減らない)。
# 実物の軸受けと同じように減速させるための小さな値。
# BASE_DIRECT_DRIVE=1 (既定) では台車は直接駆動なので走行に影響しないが、
# BASE_DIRECT_DRIVE=0 (物理車輪駆動) では走行抵抗になるので控えめにする。
_BASE_PASSIVE_WHEEL_DAMPING = max(
    0.0, _env_float('BASE_PASSIVE_WHEEL_DAMPING', 0.001))
_BASE_WHEEL_DRIVE_MAX_FORCE = max(
    0.0, _env_float('BASE_WHEEL_DRIVE_MAX_FORCE', 10.0))
_BASE_STEER_DRIVE_DAMPING = max(
    0.0, _env_float('BASE_STEER_DRIVE_DAMPING', 5.0))
_BASE_STEER_DRIVE_MAX_FORCE = max(
    0.0, _env_float('BASE_STEER_DRIVE_MAX_FORCE', 5.0))
_BASE_DIRECT_JOINT_DAMPING = max(
    0.0, _env_float('BASE_DIRECT_JOINT_DAMPING', 1.0))
_BASE_DIRECT_JOINT_MAX_FORCE = max(
    0.0, _env_float('BASE_DIRECT_JOINT_MAX_FORCE', 1.0))
_BASE_DIRECT_ROOT_DAMPING = max(
    0.0, _env_float('BASE_DIRECT_ROOT_DAMPING', 2.0))
# 駐車ブレーキ: 台車が「止まっているべき」間だけ姿勢を固定して腕の反力に
# 流されないようにする (実機のタイヤ摩擦/ブレーキの代役)。直接駆動は摩擦 0 で
# 走るため、速度ゼロ指令だけでは 1 物理ステップ内の反力を止められない。
_BASE_BRAKE_ENABLED = _env_bool('BASE_BRAKE', True)
# ブレーキを掛けてよい残差の上限。これより離れているときは従来の速度制御で寄せる。
_BASE_BRAKE_ENGAGE_LINEAR = max(
    0.0, _env_float('BASE_BRAKE_ENGAGE_LINEAR', 0.05))
_BASE_BRAKE_ENGAGE_ANGULAR = max(
    0.0, _env_float('BASE_BRAKE_ENGAGE_ANGULAR', 0.10))
# ブレーキ中に残差を詰める速さ (瞬間移動を避ける)。
_BASE_BRAKE_CREEP_SPEED = max(
    0.0, _env_float('BASE_BRAKE_CREEP_SPEED', 0.05))
_BASE_BRAKE_CREEP_ANGULAR = max(
    0.0, _env_float('BASE_BRAKE_CREEP_ANGULAR', 0.15))
# 平面ジョイントのドライブ(バネ・ダンパ)のゲイン。台車の保持と走行の両方を
# これで行う。剛性を上げすぎると 60Hz の物理で不安定になるので、ロボット総質量
# 123kg に対し固有振動数 ~40rad/s に収まる値を既定にする。
# 減衰は走行時に抵抗として働くので、C/K 分だけ目標位置を先出しして打ち消す。
_BASE_BRAKE_K = max(0.0, _env_float('BASE_BRAKE_K', 200000.0))
_BASE_BRAKE_C = max(0.0, _env_float('BASE_BRAKE_C', 20000.0))
_BASE_BRAKE_MAX_FORCE = max(0.0, _env_float('BASE_BRAKE_MAX_FORCE', 20000.0))
_BASE_BRAKE_ANGULAR_K = max(0.0, _env_float('BASE_BRAKE_ANGULAR_K', 50000.0))
_BASE_BRAKE_ANGULAR_C = max(0.0, _env_float('BASE_BRAKE_ANGULAR_C', 5000.0))
_BASE_BRAKE_MAX_TORQUE = max(
    0.0, _env_float('BASE_BRAKE_MAX_TORQUE', 5000.0))
# reset_world 直後に台車を強制固定する時間 (秒)。
_BASE_RESET_HOLD_TIME = max(0.0, _env_float('BASE_RESET_HOLD_TIME', 2.0))
# 位置サーボの目標が実位置から離れてよい上限 (アンチワインドアップ)。
_BASE_SETPOINT_MAX_LAG = max(
    0.0, _env_float('BASE_SETPOINT_MAX_LAG', 0.02))
_BASE_SETPOINT_MAX_LAG_ANGULAR = max(
    0.0, _env_float('BASE_SETPOINT_MAX_LAG_ANGULAR', 0.05))
# 平面ジョイントのドライブで台車を保持・駆動する。0 にすると素の速度制御に戻る。
_BASE_JOINT_BRAKE = _env_bool('BASE_JOINT_BRAKE', True)
# 直接駆動時は、平滑化した速度指令を姿勢へ積分し、衝突形状のない kinematic
# anchor を動かす。base_footprint は anchor へ固定するため、内部関節の物理を
# 保ったまま速度書き込みと位置ドライブの競合を避けられる。物理車輪駆動を選んだ
# ときは anchor を作らない。
_BASE_KINEMATIC_DRIVE = (
    _BASE_DIRECT_DRIVE and _env_bool('BASE_KINEMATIC_DRIVE', True))
# 走行中に articulation root の剛体速度を直接書き込むか。
# kinematic 駆動時は「anchor を跳ばした分と同じ速度」を書いて位置と速度を整合
# させるために使う (これを止めると FixedJoint の逆向きインパルスで arm_flex が
# 振られる)。非 kinematic 時は 0 にすると平面ジョイントのドライブ単独になる。
_BASE_VELOCITY_WRITE = _env_bool('BASE_VELOCITY_WRITE', True)
# 純回転のときだけ速度書き込みを省く (回転は角度ドライブ単独で出す)。
_BASE_ROT_NO_VELOCITY_WRITE = _env_bool('BASE_ROT_NO_VELOCITY_WRITE', False)
# kinematic anchor は衝突形状を持たず、base_footprint を breakForce=1e20 の
# FixedJoint で溶接するので、台車は実効的に無限質量になり壁の接触拘束が勝てない
# (壁をすり抜け、PhysX が発散して odom が NaN 化する)。そこで anchor を進める
# 前に PhysX のシーンクエリで球スイープを掛け、貫通する分だけ目標をクランプする。
_BASE_KINEMATIC_COLLISION = _env_bool('BASE_KINEMATIC_COLLISION', True)
# スイープに使う台車の半径 [m]。HSR の台車は半径 0.23m 前後で、前後バンパーを
# 含めた外周を覆うため少し大きめを既定にする。
_BASE_COLLISION_RADIUS = max(
    0.0, _env_float('BASE_COLLISION_RADIUS', 0.24))
# スイープ球の中心高さ [m]。球の下端 (中心 - 半径) が床より上に来る値にする。
# 既定 0.25 なら半径 0.24 の球の下端が z=0.01 になり、床を拾わない。
_BASE_COLLISION_HEIGHT = max(
    0.0, _env_float('BASE_COLLISION_HEIGHT', 0.25))
# 障害物の手前に残す余裕 [m]。0 にすると接触状態で止まる。
_BASE_COLLISION_MARGIN = max(
    0.0, _env_float('BASE_COLLISION_MARGIN', 0.01))
# kinematic anchor をどう動かすか。
#   'usd' : anchor prim の USD xform を毎ステップ書き、PhysX に kinematic target
#           として解釈させる。PhysX が「1ステップかけて目標へ動く」扱いにするので
#           anchor 自身が速度を持ち、FixedJoint で繋がった articulation も滑らかに
#           運ばれる。
#   'dc'  : dc.set_rigid_body_pose で瞬間移動させる (= PxRigidActor::setGlobalPose)。
#           anchor の速度は 0 のままなので、FixedJoint が位置誤差を毎ステップ
#           インパルスで埋めることになり、その衝撃が腕まで伝わって arm_flex が
#           落ちる (実測 1.493 rad)。
# usd が効かない環境 (USD→PhysX の同期が無効など) では台車が動かなくなるので、
# 追従できていないことを検出したら自動的に dc へ戻す。
_BASE_KINEMATIC_ANCHOR_MODE = (
    os.environ.get('BASE_KINEMATIC_ANCHOR_MODE', 'usd').strip().lower()
    or 'usd')
# arm_flex のドライブ力上限 [N*m]。重力たわみを支えるだけなら 100 で足りるが、
# 台車を運ぶ拘束から入る外乱に負けると、maxVelocity=1.2rad/s で飽和したまま
# 落ち続ける。余裕を持たせる。台車は kinematic に位置決めされているので、
# 反力が増えても車体は揺すられない。
_ARM_FLEX_MAX_FORCE = max(
    1.0, _env_float('ARM_FLEX_MAX_FORCE', 300.0))
# 物体が落ち着いた頃に実位置をログへ出す (把持目標を決めるのに使う)。
_OBJECT_POSE_REPORT = _env_bool('OBJECT_POSE_REPORT', True)
_OBJECT_POSE_REPORT_AT = max(
    0.0, _env_float('OBJECT_POSE_REPORT_AT', 5.0))
# 台車の状態を定期的にログへ出す (解析用)。
_BASE_DIAG = _env_bool('BASE_DIAG', False)
# 起動時に物理パラメータ一式をログへ出す (解析用)。
_BASE_PHYSICS_REPORT = _env_bool('BASE_PHYSICS_REPORT', True)
# レンダリングされたフレームのうち何枚に 1 枚を ROS へ publish するか (0 = 毎フレーム)。
# 既定 0: RENDER_EVERY_N_STEPS=2 (=30 Hz レンダリング) と組み合わせて、実機の HSR と同じ
# 30 Hz で /head_rgbd_sensor/* を出す。物理と全身制御は常に 60 Hz のまま。
# 以前は 2 (3 枚に 1 枚 = 5 Hz) だったが、これがそのまま
# /head_rgbd_sensor/reconsted/points が 4 Hz しか出ない原因だった。
# RTF が落ちるようなら 1 (=15 Hz) に戻す。
_CAMERA_FRAME_SKIP = max(0, _env_int('CAMERA_FRAME_SKIP', 0))
# head_l / head_r ステレオ (1280x960 x2) はレンダープロダクトのピクセル予算の 73% を
# 占めるのに、実測で subscriber は 0 だった。RGBD を 30 Hz で出す余力を作るため既定では
# 作らない。ステレオ画像が要るタスクのときだけ ENABLE_STEREO_CAMERAS=1 で復活させる。
_ENABLE_STEREO_CAMERAS = _env_bool('ENABLE_STEREO_CAMERAS', False)
