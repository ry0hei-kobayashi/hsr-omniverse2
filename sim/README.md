# sim/ — Isaac Sim 側

- `hsr_omniverse2_sim/`: Isaac の Python (3.11) で動く本体。`Dockerfile.isaacsim` が `/app/sim` に置いて
  `pip install -e` する。compose は `../sim:/app/sim` を bind mount するので、編集は再ビルド無しで反映される
  (`pyproject.toml` を変えたときだけ `make build`)。
  - `launch_isaacsim.py` (エントリポイント: `python.sh -m hsr_omniverse2_sim.launch_isaacsim`)
  - `robot/{hsr,hsrc_ex}` `env/` `task/` `samples/` `tools/` (Isaac の Python で動くツール)
  - `paths.py`: `REPO_ROOT` (固定深さ。`HSR_OMNIVERSE_ROOT` で上書き可) と `io/` 配下の `CONFIGS_DIR` / `USD_DIR` / `WORLDS_DIR`
- `tools/`: ホスト側で使う補助 (`apply_tune.py`, `gen_cyclonedds_pc.sh`, `isaac_localhost_mode.sh`)。パッケージ外。
