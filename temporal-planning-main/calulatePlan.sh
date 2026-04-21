#!/bin/bash

set -e

SUFFIX=${1:-1}

cd ~/catkin_ws/src
source temporal-planning-main/bin/activate
cd temporal-planning-main/temporal-planning/

if python2.7 bin/plan.py stp-2 \
    ~/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/domain/PDDL_domain_${SUFFIX}.pddl \
    ~/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/problem/PDDL_problem_${SUFFIX}.pddl \
    > /dev/null >&1; then
    cat tmp_sas_plan.1
else
    echo "Planning failed" >&2
    exit 1
fi

