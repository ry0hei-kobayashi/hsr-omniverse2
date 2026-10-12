"""リポジトリ内の固定パス。

このパッケージは editable install されるので __file__ は常にソースツリーの中を指す:
  <repo>/sim/hsr_omniverse2_sim/paths.py  → REPO_ROOT = <repo>   (ホスト、dev モード: /app/sim を bind mount)
  /app/sim/hsr_omniverse2_sim/paths.py    → REPO_ROOT = /app     (イメージ焼き込み)
深さは固定なので探索はしない。別の場所を指したいときだけ環境変数 HSR_OMNIVERSE_ROOT で上書きする。

データは io/ にまとめてある (compose が ../io を /app/io に mount、イメージにも ADD 済み):
  io/configs/        placement.yaml / dressing.yaml / tasks/<TASK>/ / textures/
  io/models/usd/     hsrb/ wrs_models/ (submodule)、isaac_offline/、my_models/、unknown_objects/ …
  io/models/worlds/  *.world
  io/recordings/     競技モードの録画・tune の出力 (コンテナ内は /recordings に rw mount)
"""
import os
from pathlib import Path

REPO_ROOT = os.environ.get('HSR_OMNIVERSE_ROOT') or str(Path(__file__).resolve().parents[2])
IO_ROOT = os.path.join(REPO_ROOT, 'io')
CONFIGS_DIR = os.path.join(IO_ROOT, 'configs')
USD_DIR = os.path.join(IO_ROOT, 'models', 'usd')
WORLDS_DIR = os.path.join(IO_ROOT, 'models', 'worlds')
