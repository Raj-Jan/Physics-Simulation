# Physics-Simulation

Exact solution for Lagrange method derivation is aviable at (chapter 2):
https://www.researchgate.net/profile/Paulo_Flores3/publication/230554861_Kinematics_and_Dynamics_of_Multibody_Systems_with_Imperfect_Joints_Models_and_Case_Studies/links/5602e66808ae460e2704b23c.pdf

Implemetation uses basic linear algebra operation using matrices.
For inverting square matrix Gaussian elimination algorithm was used.
The solution is unstable and provides to failures when given system with no freedom.

Paper explaining another implemented method:
http://mech.vub.ac.be/multibody/publications/full_texts/ECCOMAS_2003_Naudet.pdf

Only chain like structure is implemented, but generalization to tree-like topologies and closed-loop chains is possible.

