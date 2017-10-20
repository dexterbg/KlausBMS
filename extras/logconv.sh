#!/bin/bash

log=$1
base=$(basename $log .txt)
csv=${2:-$base.csv}
ods=$base.ods

echo "Konvertiere $log ..."

if [ ! -f $log ] ; then
  echo "Abbruch: '$log' nicht gefunden!"
  exit 1
fi

conv() {
  n=1
  while true ; do
    line=""
    for i in {1..6} ; do
      read col || return
      line+=$col
    done
    echo $n $line
    n=$((n + 1))
  done
}

echo \
  '#;soc;-;vpack;-;drvpwr;-;chgcur;-;state;soc_volt;-;curr;-;recpwr;-;temp_chg;-;error;' \
  'soc_coul;-;avail_ah;-;temp_f;-;cmin_soc;-;cdif;-;' \
  'soh;-;cap_ah;-;temp_r;-;cmax_soc;-;' \
  'c0;c1;c2;c3;c4;c5;c6;c7;c8;c9;c10;c11;c12;c13;c14;c15' \
  > $csv

cat $log | recode -f /CRLF | grep '^|' | conv \
  | sed -e 's:Cc ||:Cc |-|:g' -e 's:|[<>]:|:g' -e 's:[| ][| ]*:;:g' -e 's:;$::' \
  >> $csv

echo "CSV: $csv"

#unoconv -d spreadsheet -f ods -i 59,34,UTF8,,,1033 $csv
soffice --headless --convert-to ods --infilter=CSV:59,34,UTF8,,,1033 $csv

echo "ODS: $ods"

exit 0
