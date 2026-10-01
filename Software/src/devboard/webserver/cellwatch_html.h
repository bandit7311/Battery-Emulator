#ifndef CELLWATCH_H
#define CELLWATCH_H

#include <WString.h>

/**
 * @brief Replaces placeholder with content section in web page. Renders the Cellwatch diagnostic
 * page: polls /cellwatchStatus repeatedly and appends a new row only when a fresh sample has
 * actually arrived (change detection on the sample counter), no fixed reload timer.
 *
 * @param[in] var
 *
 * @return String
 */
String cellwatch_processor(const String& var);

#endif
