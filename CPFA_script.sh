#!/bin/bash
for i in $(seq 1 100); do
    argos3 -c experiments/Clustered_CPFA_r64_tag512_16by16.xml -z
done
