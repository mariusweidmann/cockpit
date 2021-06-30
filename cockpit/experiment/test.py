#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 Marius Weidmann <mweidmann@physik.uni-bielefeld.de>
##
## This file is part of Cockpit.
##
## Cockpit is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Cockpit is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Cockpit.  If not, see <http://www.gnu.org/licenses/>.

""" test experiment to test if hardware tiggering of a camera works."""

from cockpit.experiment import actionTable
from cockpit.experiment import experiment

import decimal
import math

EXPERIMENT_NAME = "Hardware Trigger Test"

class TestExperiment(experiment.Experiment):
    ### Create the ActionTable needed to run the test.
    ### We simply give 20 Hardware Triggers to the camera.

    def generateActions(self):
        table = actionTable.ActionTable()
        curTime = 0

        for i in range(20):
            for cameras, lightTimePairs in self.exposureSettings:
                curTime = self.expose(curTime, cameras, lightTimePairs, table)
                # Advance the time very slightly so that all exposures
                # are strictly ordered.
                curTime += decimal.Decimal('1e-10')

        return table



## A consistent name to use to refer to the class itself.
EXPERIMENT_CLASS = TestExperiment