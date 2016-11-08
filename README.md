# AESIR
AESIR Aerodynamics Repository


This trajectory model was used for the Fundaments of Spaceflight course of the Aerospace program. It is modeled using Tsiolkovsky rocket
rocket equation and the drag is assumed to be constant. We need to model the drag and consider the effects of change in center of gravity
and center of pressure (pitch dynamics) behaviour for a better model. 


stage1.m is the main executable file which uses the solver (ODE45) files TE.m, TE2.m, TE3.m & TE4.m. The relevant plots can be generated on running the main file.
