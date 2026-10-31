#!/usr/bin/env bash
# Build the existing Foxy RMW source with optimization in an isolated prefix.
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "用法：bash $0 RMW包源码目录 CycloneDDS的cmake目录 [输出目录]" >&2
  exit 2
fi
source "$SCRIPT_DIR/g1_env.sh"
g1_load_ros
[[ "$(uname -m)" == aarch64 ]] || { g1_error '此构建入口要求 PC2 aarch64'; exit 1; }
RMW_SOURCE="$(cd -- "$1" && pwd)"
DDS_CMAKE="$(cd -- "$2" && pwd)"
RMW_OUTPUT="${3:-$G1_WS/runtime/rmw_release}"
[[ -f "$RMW_SOURCE/src/rmw_node.cpp" && -f "$RMW_SOURCE/package.xml" ]] || {
  g1_error '请指定包含 src/rmw_node.cpp 的 rmw_cyclonedds_cpp 包目录'; exit 1;
}
[[ -f "$DDS_CMAKE/CycloneDDSConfig.cmake" ]] || { g1_error '找不到 CycloneDDSConfig.cmake'; exit 1; }
mkdir -p "$RMW_OUTPUT"
RMW_OUTPUT="$(cd -- "$RMW_OUTPUT" && pwd)"
# Refuse silently reusing a cache for a different source, DDS or install prefix.
python3 - "$RMW_OUTPUT" "$RMW_SOURCE" "$DDS_CMAKE" <<'PY'
from pathlib import Path
import sys
root, source, dds = sys.argv[1:]
cache = Path(root) / 'build/CMakeCache.txt'
if cache.exists():
    values = dict(line.split('=', 1) for line in cache.read_text().splitlines()
                  if '=' in line and not line.startswith(('#', '//')))
    expected = {'CMAKE_HOME_DIRECTORY:INTERNAL': source,
                'CMAKE_INSTALL_PREFIX:PATH': root + '/install',
                'CycloneDDS_DIR:PATH': dds}
    for key, value in expected.items():
        if values.get(key) != value:
            raise SystemExit('构建缓存不匹配，请选择新的输出目录: ' + key)
PY
cmake -S "$RMW_SOURCE" -B "$RMW_OUTPUT/build" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_INSTALL_PREFIX="$RMW_OUTPUT/install" -DCycloneDDS_DIR="$DDS_CMAKE"
cmake --build "$RMW_OUTPUT/build" --parallel "${G1_BUILD_JOBS:-2}"
cmake --install "$RMW_OUTPUT/build"
[[ -s "$RMW_OUTPUT/install/lib/librmw_cyclonedds_cpp.so" ]] || {
  g1_error '未生成 RMW 库；不能使用跳过 CycloneDDS 的空安装'; exit 1;
}
cat > "$RMW_OUTPUT/install/local_setup.bash" <<'EOF'
#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/share/rmw_cyclonedds_cpp/local_setup.bash"
EOF
python3 - "$RMW_OUTPUT" "$RMW_SOURCE" "$DDS_CMAKE" <<'PY'
from pathlib import Path
import hashlib
import json
import sys
root, source, dds = map(Path, sys.argv[1:])
library = root / 'install/lib/librmw_cyclonedds_cpp.so'
sources = sorted(p for p in source.rglob('*') if p.is_file()
                 and (p.suffix in ('.cpp', '.hpp', '.h') or p.name in ('CMakeLists.txt', 'package.xml')))
record = {'source': str(source), 'dds_cmake': str(dds), 'build_type': 'Release',
          'source_sha256': {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in sources},
          'library': str(library), 'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
          'compiler_flags': (root / 'build/CMakeFiles/rmw_cyclonedds_cpp.dir/flags.make').read_text()}
(root / 'build_receipt.json').write_text(json.dumps(record, indent=2) + '\n')
PY
echo "[OK] 构建完成；不会自动切换现场环境。待性能及通信验证后设置："
echo "G1_RMW_SETUP=$RMW_OUTPUT/install/local_setup.bash"
echo "G1_DDS_LIB_DIR=$G1_DDS_LIB_DIR"
