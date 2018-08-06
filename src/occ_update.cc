// Update occupancy with new scan data (source)

#include "occ_update.hh"

// Update occupancy hash map with data per pose
void vox_update(spp::sparse_hash_map<ind, opp_data> & opp,
                spp::sparse_hash_map<ind, occ_data> & occ,
                spp::sparse_hash_map<ind, free_unk_data> & freev,
                spp::sparse_hash_map<ind, free_unk_data> & unk,
                int pose_ind
                )
{

  spp::sparse_hash_map <ind, opp_data> ::iterator it;
  for(it = opp.begin(); it != opp.end(); ++it)
  {
    ind cind = {it->first.x,
                it->first.y,
                it->first.z};

    ++occ[cind].hits;
    occ[cind].intensity = it->second.intensity;
  }

  // Reset temporary occupancy map
  opp.clear();

  float mean_probability = prob_update(occ,
                                       pose_ind
                                       );

  // Retrieve bounding box
  corners bounds(occ,
                 mean_probability
                 );

  bounds.even_out();

  // Update unkown voxels
  unk.clear();
  for(int x_i = bounds.min_x; x_i <= bounds.max_x; ++x_i)
  {
    for(int y_i = bounds.min_y; y_i <= bounds.max_y; ++y_i)
    {
      for(int z_i = bounds.min_z; z_i <= bounds.max_z; ++z_i)
      {
        if(freev.count( {x_i, y_i, z_i} ) == 0)
        {
          if(occ.count( {x_i, y_i, z_i} ) == 0)
          {
            // Store unknown voxel indecies
            ++unk[ {x_i, y_i, z_i} ].hits;
          }
        }
      }
    }
  }

}

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Update occupancy masking with probabilistic calculation
float prob_update(spp::sparse_hash_map<ind, occ_data> & occ,
                  int pose_ind
                  )
{

  float mean_probability = 0.0f;
  spp::sparse_hash_map <ind, occ_data> ::iterator it;
  for(it = occ.begin(); it != occ.end(); ++it)
  {
    it->second.probability = (float)it->second.hits / (float)(pose_ind+1);
    if(it->second.probability == 1) { it->first.print(); }
    mean_probability = mean_probability + it->second.probability;
  }

  mean_probability = mean_probability / occ.size();

  // Cull occupied space according to occupancy probability
  spp::sparse_hash_map <ind, occ_data> ::iterator it_cull;
  float threshold = 2.0f;

  for(it_cull = occ.begin(); it_cull != occ.end(); ++it_cull)
  {
    ind cind = {it_cull->first.x,
                it_cull->first.y,
                it_cull->first.z};

    if((occ[cind].probability > (threshold * mean_probability)))
    {
      occ[cind].mask = true;
    }
    else if((occ[cind].probability < (threshold * mean_probability)))
    {
      occ[cind].mask = false;
    }
  }

  return mean_probability;

}
