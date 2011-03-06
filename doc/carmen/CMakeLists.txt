remake_add_documentation(SOURCE man man3/*.gz)
remake_add_documentation(DOXYGEN Doxyfile.remake
  INPUT ${REMAKE_PROJECT_SOURCE_DIR} .
  PATTERNS *.h *.hpp *.dox TYPES html)
