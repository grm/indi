
#####################################
########## SPECTROGRAPH GROUP #######
#####################################

####################################################################################################
#
# Component   : Shelyak USIS driver
# Supported OS: Linux, BSD, MacOS, Cygwin
#
#################################################################################################
set( shelyak_usis_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/shelyak/indi_shelyak_usis.cpp
)

add_executable(shelyak_usis ${shelyak_usis_SRCS})
target_link_libraries(shelyak_usis indidriver ${JSONLIB})

install(TARGETS shelyak_usis RUNTIME DESTINATION bin)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/shelyak/shelyak_boards.json DESTINATION ${DATA_INSTALL_DIR})

##################################################################################################
##################################################################################################
set( spectracyber_SRCS
  spectracyber.cpp
)

add_executable(indi_spectracyber ${spectracyber_SRCS})
target_link_libraries(indi_spectracyber indidriver ${JSONLIB})

install(TARGETS indi_spectracyber RUNTIME DESTINATION bin)

#################################################################################
