"""io/configs/tasks/<TASK>/ の設定読み込みと環境変数による上書き (launch_isaacsim.py から分離)。

純粋関数のみ (Isaac / ROS に依存しない)。使い方と優先順位は launch_isaacsim.py の「タスク選択」の節を参照。
"""
import os

import yaml

from hsr_omniverse2_sim.paths import CONFIGS_DIR


TASK_DEFAULTS = {
    'time_limit': 0,
    'arena_cameras': False,
    'placement_seeds': 'placement.seed{n}.yaml',
}


def load_task_config(task):
    """io/configs/tasks/<task>/ を探し、(task_dir, task.yaml の辞書, placement.yaml のパス) を返す。

    task が空なら (None, {}, None)。フォルダが無ければ FileNotFoundError。
    task.yaml が無いときは警告して空の辞書 (world/dressing は既定)。
    placement.yaml はタスクフォルダにあればそのパス、無ければ None (各ローダーの既定)。
    """
    if not task:
        return None, {}, None
    task_dir = next(
        (
            d
            for d in (
                os.path.join(CONFIGS_DIR, 'tasks', task),
            )
            if os.path.isdir(d)
        ),
        None,
    )
    if task_dir is None:
        raise FileNotFoundError(
            f"TASK='{task}' のフォルダが見つかりません。io/configs/tasks/{task}/ を作ってください。"
        )
    print(f"[task] selected TASK='{task}' dir={task_dir}")

    task_yaml = os.path.join(task_dir, 'task.yaml')
    if os.path.isfile(task_yaml):
        with open(task_yaml) as f:
            tcfg = yaml.safe_load(f) or {}
    else:
        tcfg = {}
        print(f'[task] WARNING: {task_yaml} が無いので world/dressing は既定を使います。')

    # placement.yaml はタスクフォルダにあればそれを優先 (無ければ共通の既定)。
    placement = os.path.join(task_dir, 'placement.yaml')
    if os.path.isfile(placement):
        print(f'[task] placement: {placement}')
    else:
        placement = None
        print('[task] placement: タスク個別が無いので共通の io/configs/placement.yaml を使用')
    return task_dir, tcfg, placement


def resolve_task_time(task_time_limit):
    """競技時間 [秒] を決める。環境変数 TASK_TIME があれば task.yaml の time_limit を上書き。

    `make up TASK=<name> TIME=600` が TASK_TIME=600 として届く。空なら task.yaml の値。
    数値として読めない値は警告して task.yaml の値に戻す。負の値は 0 (無制限) 扱い。
    """
    try:
        value = float(task_time_limit or 0)
    except (TypeError, ValueError):
        print(f'[task] WARNING: task.yaml の time_limit={task_time_limit!r} を数値として'
              f'読めません。0 (無制限) にします。', flush=True)
        value = 0.0
    env = os.environ.get('TASK_TIME', '').strip()
    if env:
        try:
            value = float(env)
            print(f'[task] TASK_TIME={env} で task.yaml の time_limit を上書き', flush=True)
        except ValueError:
            print(f"[task] WARNING: TASK_TIME={env!r} を数値として読めません。"
                  f'task.yaml の time_limit={value:g} を使います。', flush=True)
    return max(value, 0.0)


def resolve_camera_tune():
    """調整モードか (環境変数 CAMERA_TUNE)。`make tune TASK=<name>` が 1 を渡す。"""
    return os.environ.get('CAMERA_TUNE', '').strip().lower() in (
        '1', 'true', 'yes', 'on')


def resolve_placement_seed(task, task_dir, pattern, base_placement_path):
    """objects: の配置に使う placement ファイルを決める。

    環境変数 PLACEMENT_SEED=<n> (make up TASK=<name> SEED=<n>) があれば
    io/configs/tasks/<task>/<pattern の {n} を n に置換> を返す。TASK が無い・n が整数でない・
    ファイルが無いときは理由を明示して止める (黙って既定に落ちると配置ミスに気付けない)。
    無指定なら base_placement_path (タスクの placement.yaml, または None=共通の既定)。
    """
    seed_env = os.environ.get('PLACEMENT_SEED', '').strip()
    if not seed_env:
        return base_placement_path
    if not task:
        raise RuntimeError(
            f'PLACEMENT_SEED={seed_env} が指定されましたが TASK がありません。'
            'Seed ファイルは io/configs/tasks/<TASK>/ から探すので、TASK=<name> も指定してください。'
        )
    try:
        seed = int(seed_env)
    except ValueError:
        raise ValueError(
            f'PLACEMENT_SEED={seed_env!r} は整数ではありません (例: SEED=2)。'
        ) from None
    pattern = str(pattern or TASK_DEFAULTS['placement_seeds'])
    if '{n}' not in pattern:
        raise ValueError(
            f"TASK='{task}' の task.yaml の placement_seeds={pattern!r} に {{n}} がありません。"
            "例: placement_seeds: placement.seed{n}.yaml"
        )
    seed_path = os.path.join(task_dir, pattern.replace('{n}', str(seed)))
    if not os.path.isfile(seed_path):
        raise FileNotFoundError(
            f"TASK='{task}' の Seed {seed} の配置ファイルが見つかりません: {seed_path}\n"
            f'task.yaml の placement_seeds ({pattern}) に合う名前で '
            f'io/configs/tasks/{task}/ に置いてください。'
        )
    print(f'[task] PLACEMENT_SEED={seed} -> objects: は {seed_path} を使用 '
          '(robot/people/furniture/arena_cameras は placement.yaml のまま)', flush=True)
    return seed_path


def resolve_render_every_n_steps(default=2):
    """物理 60 Hz のうち何ステップに 1 回描画するか (環境変数 RENDER_EVERY_N_STEPS)。

    描画したフレームだけがカメラの publish 候補になるので、ここが RGB-D の上限レートを決める:
      カメラ publish [Hz] = 60 / RENDER_EVERY_N_STEPS / (CAMERA_FRAME_SKIP + 1) * RTF
    既定 2 = 30 Hz 描画。RTF が 1.0 を保てないときは 3 や 4 に。物理と全身制御は常に 60 Hz。
    """
    raw = os.environ.get('RENDER_EVERY_N_STEPS', '').strip()
    if not raw:
        return default
    try:
        return max(1, int(raw))
    except ValueError:
        print('[render] WARNING: RENDER_EVERY_N_STEPS=%r is invalid; using %d'
              % (raw, default), flush=True)
        return default
