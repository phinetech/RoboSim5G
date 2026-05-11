SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

export UE1_IMSI="001010000000101"
export UE1_KEY="8baf473f2f8fd09487cccbd7097c6862"
export UE1_OPC="8e27b6af0e692e750f32667a3b14605d"
export UE1_DNN="internet"
export UE1_SST="1"
export UE1_SD="000001"

sudo "$SCRIPT_DIR/open5gs-dbctl" add_ue_with_slice \
  "$UE1_IMSI" \
  "$UE1_KEY" \
  "$UE1_OPC" \
  "$UE1_DNN" \
  "$UE1_SST" \
  "$UE1_SD" || true