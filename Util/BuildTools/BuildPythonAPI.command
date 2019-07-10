#! /bin/bash

source $(dirname "$0")/Environment.sh

export CC=clang
export CXX=clang++

# ==============================================================================
# -- Parse arguments -----------------------------------------------------------
# ==============================================================================

DOC_STRING="Build and package CARLA Python API."

USAGE_STRING="Usage: $0 [-h|--help] [--rebuild] [--py2] [--py3] [--clean]"

REMOVE_INTERMEDIATE=false
BUILD_FOR_PYTHON2=false
BUILD_FOR_PYTHON3=false

while true; do
  case "$1" in
    --rebuild )
      REMOVE_INTERMEDIATE=true;
      BUILD_FOR_PYTHON2=true;
      BUILD_FOR_PYTHON3=true;
      shift ;;
    --py2 )
      BUILD_FOR_PYTHON2=true;
      shift ;;
    --py3 )
      BUILD_FOR_PYTHON3=true;
      shift ;;
    --clean )
      REMOVE_INTERMEDIATE=true;
      shift ;;
    -h | --help )
      echo "$DOC_STRING"
      echo "$USAGE_STRING"
      exit 1
      ;;
    * )
      if [ ! -z "$1" ]; then
        echo "Bad argument: '$1'"
        echo "$USAGE_STRING"
        exit 2
      fi
      break ;;
  esac
done

if ! { ${REMOVE_INTERMEDIATE} || ${BUILD_FOR_PYTHON2} || ${BUILD_FOR_PYTHON3}; }; then
  fatal_error "Nothing selected to be done."
fi

pushd "${CARLA_PYTHONAPI_ROOT_FOLDER}" >/dev/null

# ==============================================================================
# -- Clean intermediate files --------------------------------------------------
# ==============================================================================

if ${REMOVE_INTERMEDIATE} ; then

  log "Cleaning intermediate files and folders."

  rm -Rf build dist carla.egg-info source/carla.egg-info

  find source -name "*.dylib" -delete
  find source -name "__pycache__" -type d -exec rm -r "{}" \;

fi

# ==============================================================================
# -- Build API -----------------------------------------------------------------
# ==============================================================================

if ${BUILD_FOR_PYTHON2} ; then

  log "Building Python API for Python 2."

  /usr/bin/env python2 setup.py bdist_egg

fi

if ${BUILD_FOR_PYTHON3} ; then

  log "Building Python API for Python 3."

  /usr/bin/env python3 setup.py bdist_egg

fi

# ==============================================================================
# -- ...and we are done --------------------------------------------------------
# ==============================================================================

popd >/dev/null

log "Success!"