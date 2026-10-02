#ifndef SIMULATOR_HTML_H
#define SIMULATOR_HTML_H

#include <WString.h>

/**
 * @brief Renders the /simulator diagnostic page: one checkbox per cyclic CAN signal
 * (RenaultTwingoGen1Battery::sim_signals), grouped by send interval, with an I/P/A legend
 * and a note on signals whose real source is the BMS itself (not the EVC).
 *
 * @param[in] var
 * @return String
 */
String simulator_processor(const String& var);

#endif
