#!/bin/bash

exe=$1
compiler=$2
flags=$3

if ! hash datamash 2>/dev/null; then
  echo "This script requires datamash"
  echo "On Ubuntu: apt-get install datamash"
  echo "On Mac OS X (Homebrew): brew install datamash"
  exit 1
fi

if [[ $# != 3 ]]; then
  echo "This script should not be used directly, instead run \`make benchmarks\`"
  exit 2
fi

echo "Running benchmarks"
results="${exe}_${compiler}.csv"
tmp="out"

first=1
for precision in float double; do
  for flag in $(echo ${flags} | tr ";" "\n"); do
    echo "Compiler: ${compiler}"
    echo "Precision: ${precision}"
    echo "Flag: ${flag}"
    ./${exe}_${precision}_${flag} -t ${tmp}
    if [ ${first} -eq 1 ]; then
      echo -n ",,," > ${results}
      cat ${tmp} | awk -F ',' -e '{ if ($2 == "Proposed") print $1 }' | datamash -t "," transpose >> ${results}
      first=0
    fi
    echo -n "${precision},${compiler},${flag}," >> ${results}
    cat ${tmp} | awk -F ',' -e '{ if ($2 == "Proposed") { printf("%.2f ", $7); if (index($1, "P_")) { printf("(%.0f ns)\n", $8 * 1000) } else { printf("(%.0f us)\n", $8) } } }' | datamash -t "," transpose >> ${results}
  done
done

echo "Results saved to: `pwd`/${results}"
