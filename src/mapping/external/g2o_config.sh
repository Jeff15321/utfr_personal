# Need to add a file to the g2o library without building the whole things (Takes a long time)

cd ./g2o/g2o

touch config.h
echo "#ifndef G2O_CONFIG_H
#define G2O_CONFIG_H

#define G2O_HAVE_OPENGL 1
#define G2O_OPENGL_FOUND 1
/* #undef G2O_OPENMP */
#define G2O_SHARED_LIBS 1
/* #undef G2O_LGPL_SHARED_LIBS */

// available sparse matrix libraries
#define G2O_HAVE_CHOLMOD 1
#define G2O_HAVE_CSPARSE 1

// logging framework available
#define G2O_HAVE_LOGGING

/* #undef G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS */

#ifdef G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS
#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 0
#else
#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 1
#endif

#define G2O_CXX_COMPILER "GNU /usr/bin/c++"

#endif" >> config.h