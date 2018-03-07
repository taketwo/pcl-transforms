include(ExternalProject)

ExternalProject_Add(celero
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/celero
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCELERO_ENABLE_EXPERIMENTS=OFF -DCMAKE_MACOSX_RPATH=ON
)

ExternalProject_Get_Property(celero INSTALL_DIR)
set(CELERO_INCLUDE_DIRS ${INSTALL_DIR}/include CACHE INTERNAL "Celero include directories")
if(APPLE)
	set(CELERO_LIBRARIES ${INSTALL_DIR}/bin/libcelero.dylib CACHE INTERNAL "Celero libraries")
elseif(UNIX)
	set(CELERO_LIBRARIES ${INSTALL_DIR}/bin/libcelero.so CACHE INTERNAL "Celero libraries")
endif()
set(CELERO_RPATH ${INSTALL_DIR}/bin CACHE INTERNAL "Celero libraries runtime folder")

ExternalProject_Add(gtest
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/googletest
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DBUILD_GTEST=ON -DBUILD_GMOCK=OFF
)

ExternalProject_Get_Property(gtest INSTALL_DIR)
set(GTEST_INCLUDE_DIRS ${INSTALL_DIR}/include CACHE INTERNAL "Gtest include directories")
set(GTEST_LIBRARIES ${INSTALL_DIR}/lib/libgtest.a pthread CACHE INTERNAL "Gtest libraries")

find_package(PCL COMPONENTS common REQUIRED QUIET)
set(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS} CACHE INTERNAL "PCL include directories")
