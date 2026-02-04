# klipper4cnc
klipper4cnc is an experimental CNC motion pipeline built on top of Klipper. Rather than acting as a traditional G-code sender, klipper4cnc implements a structured CNC control stack that mirrors how real CNC controllers work internally: modal state, arc expansion, motion primitives, streaming execution, etc. This is very much a work in progress.
