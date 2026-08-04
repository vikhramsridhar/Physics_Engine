#pragma once
#include "contact.h"
#include <vector>

// Sequential impulse resolution for one contact: normal impulse (with
// restitution) plus Coulomb friction. Uses applyImpulseAtPoint so torque
// from off-center contacts (boxes!) is handled correctly, not just spheres.
void resolveContact(Contact& c, float restitution, float friction);

// Positional (Baumgarte-style) correction to fix residual penetration
// after the velocity solve, so bodies don't sink into each other over time.
void positionalCorrection(Contact& c);

void solveContacts(std::vector<Contact>& contacts, float restitution,
                    float friction, int iterations);
