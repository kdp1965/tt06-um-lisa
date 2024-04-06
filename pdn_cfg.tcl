# Copyright 2020-2022 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

source $::env(SCRIPTS_DIR)/openroad/common/set_global_connections.tcl
set_global_connections

define_pdn_grid \
    -name stdcell_grid \
    -starts_with POWER \
    -voltage_domain CORE \
    -pins $::env(FP_PDN_VERTICAL_LAYER)

add_pdn_stripe \
    -grid stdcell_grid \
    -layer $::env(FP_PDN_VERTICAL_LAYER) \
    -width $::env(FP_PDN_VWIDTH) \
    -pitch $::env(FP_PDN_VPITCH) \
    -offset $::env(FP_PDN_VOFFSET) \
    -starts_with POWER


# Adds the standard cell rails if enabled.
if { $::env(FP_PDN_ENABLE_RAILS) == 1 } {
    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(FP_PDN_RAIL_LAYER) \
        -width $::env(FP_PDN_RAIL_WIDTH) \
        -followpins \
        -starts_with POWER

    add_pdn_connect \
        -grid stdcell_grid \
        -layers "$::env(FP_PDN_RAIL_LAYER) $::env(FP_PDN_VERTICAL_LAYER)"
}
