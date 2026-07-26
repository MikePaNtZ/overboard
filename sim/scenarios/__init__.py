"""Scripted sim scenarios for the Overboard plant model.

Each scenario is a deterministic, parameterised experiment that produces a
metrics dict + a trajectory. Scenarios are the unit the CI gate runs and the
unit the renderer films; nothing here touches the interactive viewer.
"""
