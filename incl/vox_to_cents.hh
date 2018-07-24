// Conversion & writing of voxels to real-space centers (header)

#ifndef vox_to_cents_hh
#define vox_to_cents_hh

#include <fstream>

#include <vector>
#include <algorithm>

#include <unordered_map>
#include <utility>
#include <functional>

#include "ind.hh"
#include "pt.hh"
#include "occ_data.hh"
#include "free_unk_data.hh"

// Overload for occupied voxels
void get_vox_cents(std::unordered_map <ind, occ_data> & vox,
                   std::vector<pt> & cents,
                   float resolution
                   );

// Overload for free/unk voxels
void get_vox_cents(std::unordered_map <ind, free_unk_data> & vox,
                   std::vector<pt> & cents,
                   float resolution
                   );

// Write cents to file
void write_cents(std::vector<pt> & cents,
                 std::string filename
                 );

#endif
