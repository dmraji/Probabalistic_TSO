// Conversion & writing of voxels to real-space centers (source)

#include "vox_to_cents.hh"

// Overload for occupied voxels
void get_vox_cents(std::unordered_map <ind, occ_data> & vox,
                   std::vector<pt> & cents,
                   float resolution
                   )
{
  // Reserve for centers
  cents.reserve(vox.size());
  // Construct map iterator
  std::unordered_map <ind, occ_data> ::iterator vox_iterator;
  for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
  {
    if(vox_iterator->second.mask)
    {
      cents.push_back({(vox_iterator->first.x + 0.5f) * resolution, (vox_iterator->first.y + 0.5f) * resolution, (vox_iterator->first.z + 0.5f) * resolution});
    }
  }
}

// Overload for free/unk voxels
void get_vox_cents(std::unordered_map <ind, free_unk_data> & vox,
                  std::vector<pt> & cents,
                  float resolution
                  )
{
  // Reserve for centers
  cents.reserve(vox.size());
  // Construct map iterator
  std::unordered_map <ind, free_unk_data> ::iterator vox_iterator;
  for(vox_iterator = vox.begin(); vox_iterator != vox.end(); ++vox_iterator)
  {
    cents.push_back({(vox_iterator->first.x + 0.5f) * resolution, (vox_iterator->first.y + 0.5f) * resolution, (vox_iterator->first.z + 0.5f) * resolution});
  }
}

// Write cents to file
void write_cents(std::vector<pt> & cents,
                 std::string filename
                 )
{
  std::ofstream file;
  file.open(filename+".txt");
  for(int i = 0; i < cents.size(); ++i)
  {
    file << cents[i].x << ", " << cents[i].y << ", " << cents[i].z << "\n";
  }
  file.close();
}
