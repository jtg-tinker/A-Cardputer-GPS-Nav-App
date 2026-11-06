#!/usr/bin/env bash
# Regenerates src/certs.h from the host CA store (Debian/Ubuntu ca-certificates).
set -euo pipefail
cd "$(dirname "$0")/.."
CERTS=(GTS_Root_R1 GTS_Root_R4 ISRG_Root_X1 ISRG_Root_X2 DigiCert_Global_Root_G2 USERTrust_RSA_Certification_Authority)
{
  echo '#pragma once'
  echo '// Root CAs for api.maptiler.com (GTS), tile.thunderforest.com and'
  echo '// nominatim.openstreetmap.org (ISRG), plus common fallbacks. Regenerate with scripts/gen_certs.sh.'
  echo 'static const char kRootCAs[] ='
  for c in "${CERTS[@]}"; do sed 's/.*/    "&\\n"/' "/etc/ssl/certs/$c.pem"; done
  echo '    ;'
} > src/certs.h
