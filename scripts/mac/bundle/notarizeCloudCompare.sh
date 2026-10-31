#!/bin/bash
# Notarizes a file with Apple and staples the result.
#
# Usage: notarizeCloudCompare.sh <file to submit (.zip or .dmg)> <file to staple (.app or .dmg)>
#
# Credentials, an App Store Connect API key (CI):
#   NOTARY_KEY_PATH, NOTARY_KEY_ID, NOTARY_ISSUER_ID
# or a profile saved with `xcrun notarytool store-credentials`:
#   NOTARY_KEYCHAIN_PROFILE

set -euo pipefail

if [ $# -ne 2 ]; then
	echo "Usage: $0 <file to submit (.zip or .dmg)> <file to staple (.app or .dmg)>"
	exit 1
fi

submit=$1
staple=$2

if [ -n "${NOTARY_KEYCHAIN_PROFILE:-}" ]; then
	credentials=(--keychain-profile "$NOTARY_KEYCHAIN_PROFILE")
else
	credentials=(--key "$NOTARY_KEY_PATH" --key-id "$NOTARY_KEY_ID" --issuer "$NOTARY_ISSUER_ID")
fi

result=$(xcrun notarytool submit "$submit" "${credentials[@]}" --wait --output-format json)
echo "$result"

status=$(echo "$result" | jq -r '.status')
if [ "$status" != "Accepted" ]; then
	# show why Apple rejected it
	xcrun notarytool log "$(echo "$result" | jq -r '.id')" "${credentials[@]}" || true
	exit 1
fi

xcrun stapler staple "$staple"
xcrun stapler validate "$staple"
