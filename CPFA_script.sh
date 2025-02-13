#!/bin/bash
for i in $(seq 1 50); do
   argos3 -c experiments/Clustered_CPFA_r40_tag256_8by8_TEST.xml
done
