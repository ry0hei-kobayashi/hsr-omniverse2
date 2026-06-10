#!/usr/bin/env bash
# assets/cyclonedds.pc.xml を「今このPC」向けに自動生成する。
# PCごとに変わる 2 か所(自分のNIC名 / 相手のIP)を手編集せずに済ませるためのもの。
#
# 使い方:
#   scripts/gen_cyclonedds_pc.sh 192.168.0.10            # 相手(ラップトップ)IPを渡す
#   scripts/gen_cyclonedds_pc.sh 192.168.0.10 192.168.0.11   # 複数台つなぐとき
#   ISAAC_NIC=enp3s0 scripts/gen_cyclonedds_pc.sh 192.168.0.10   # NICを明示したいとき
#
# NIC名は「相手と同じサブネット(/24)のIPを持つNIC」を自動判定する(5d_isaac_sim_mode.sh と同方式)。
# 判定順: ISAAC_NIC 明示 > ip で同subnetのNIC > ip route > python3 で /sys から直接。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SCRIPT_DIR/../assets/cyclonedds.pc.xml"

if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <相手IP> [相手IP ...]" >&2
    echo "  例: $0 192.168.0.10" >&2
    exit 1
fi
PEERS=("$@")
first_peer="${PEERS[0]}"
pfx="$(echo "$first_peer" | cut -d. -f1-3)."   # 例 "192.168.0."

# --- NIC 自動判定 ---
nic="${ISAAC_NIC:-}"
if [ -z "$nic" ]; then
    nic=$(ip -o -4 addr show 2>/dev/null | awk -v p="$pfx" 'index($4,p)==1 {print $2; exit}') || true
fi
if [ -z "$nic" ]; then
    nic=$(ip route get "$first_peer" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' | head -n1) || true
fi
if [ -z "$nic" ]; then
    # ip コマンドが無い環境向け: python3 で各NICのIPv4を直接調べ同subnetのNICを選ぶ
    nic=$(python3 - "$pfx" <<'PY' 2>/dev/null || true
import os, socket, fcntl, struct, sys
pfx = sys.argv[1]
for n in sorted(os.listdir("/sys/class/net")):
    if n == "lo":
        continue
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip = socket.inet_ntoa(fcntl.ioctl(
            s.fileno(), 0x8915, struct.pack("256s", n[:15].encode()))[20:24])
    except OSError:
        continue
    if ip.startswith(pfx):
        print(n)
        break
PY
)
fi

if [ -z "$nic" ]; then
    echo "エラー: ${first_peer}(${pfx}x) と同じサブネットのNICが見つかりません。" >&2
    echo "  ISAAC_NIC=<NIC名> を指定して再実行してください。現在のIPv4:" >&2
    ip -o -4 addr show 2>/dev/null | awk '{print "    "$2"\t"$4}' >&2 || true
    exit 2
fi

# --- Peers XML 組み立て ---
peers_xml="                <Peer Address=\"localhost\"/>"
for ip in "${PEERS[@]}"; do
    peers_xml="${peers_xml}
                <Peer Address=\"${ip}\"/>"
done

# --- 出力 ---
cat > "$OUT" <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<!-- 自動生成: scripts/gen_cyclonedds_pc.sh ${PEERS[*]}  (NIC=${nic}) -->
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="${nic}"/>
            </Interfaces>
            <AllowMulticast>false</AllowMulticast>
            <EnableMulticastLoopback>false</EnableMulticastLoopback>
            <MaxMessageSize>65500B</MaxMessageSize>
            <FragmentSize>4000B</FragmentSize>
        </General>
        <Discovery>
            <Peers>
${peers_xml}
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
        </Discovery>
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
            <SocketSendBufferSize min="10MB"/>
        </Internal>
    </Domain>
</CycloneDDS>
XML

echo "生成しました: $OUT"
echo "  NIC   = $nic"
echo "  Peers = localhost ${PEERS[*]}"
