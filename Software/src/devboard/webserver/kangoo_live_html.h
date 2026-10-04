#ifndef KANGOO_LIVE_HTML_H
#define KANGOO_LIVE_HTML_H

// Small page /kangooLive of the Renault Kangoo driver (RX only, in the car). It is a fixed page without template
// placeholders: it fetches a short text from /kangooLiveData every second and changes only the lines, so no full page
// is built for the update. Buttons call /kangooLiveAction.
extern const char kangoo_live_page[];

#endif
