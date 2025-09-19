#pragma once

#include "MeshtasticCompactNodeInfoDB.hpp"

class MeshtasticCompactFileIO {
   public:
    static const uint8_t FILEIO_VERSION = 1;
    // Save the nodedb
    static bool saveNodeDb(NodeInfoDB& db);

    // Load the nodedb
    static bool loadNodeDb(NodeInfoDB& db);
};