PROJECT (BASIC_PROJECT_ORGANIZATION)

# add sub-directory src for compilation and save the binary generated to bin under build
ADD_SUBDIRECTORY(src)

# install doc:
INSTALL(FILES COPYRIGHT README.md DESTINATION share/doc/cmake/02-basic-project-organization)
INSTALL(PROGRAMS ${PROJECT_BINARY_DIR}/bin/ run.sh DESTINATION bin)
INSTALL(DIRECTORY doc/ DESTINATION share/doc/cmake/02-basic-project-organization)
