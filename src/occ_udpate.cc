// Update occupancy with new scan data (source)

#include "occ_update.hh"

// Update occupancy hash map with data per pose
void vox_update(std::unordered_map<ind, free_unk_data> & opp,
                std::unordered_map<ind, occ_data> & occ,
                std::unordered_map<ind, free_unk_data> & freev,
                std::unordered_map<ind, free_unk_data> & unk
                int pose_ind
                )
{

  std::unordered_map <ind, free_unk_data> ::iterator it;
  for(it = opp.begin(); it != opp.end(); ++it)
  {
    ind cpt = {it->first.x, it->first.y, it->first.z};

    ++occ[cpt].hits;
    occ[cpt].probability = (float)occ[cpt].hits / (float)(pose_ind+1);
  }

  // Reset temporary occupancy map
  opp.clear();

  float mean_probability = prob_update(occ
                                       );

  // Retrieve bounding box
  corners bounds(occ,
                 mean_probability
                 );

  bounds.even_out();

  // Update unkown voxels
  box_unknown.clear();
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

// Update occupancy masking with probabilistic calculation
float prob_update(std::unordered_map<ind, occ_data> & occ
                  )
{

  float mean_probability = 0.0f;
  std::unordered_map <ind, occ_data> ::iterator it;

  for(it = occ.begin(); it != occ.end(); ++it)
  {
    mean_probability = mean_probability + it->second.probability;
  }

  mean_probability = mean_probability / occ.size();

  // Cull occupied space according to occupancy probability
  std::unordered_map <ind, occ_data> ::iterator it_cull;
  float threshold = 1.5f;

  for(it_cull = occ.begin(); it_cull != occ.end(); ++it_cull)
  {
    ind cpt = {it_cull->first.x, it_cull->first.y, it_cull->first.z};

    if((occ[cpt].probability > (threshold * mean_probability)))
    {
      occ[cpt].mask = true;
    }
    else if((occ[cpt].probability < (threshold * mean_probability)))
    {
      occ[cpt].mask = false;
    }
  }

  return mean_probability;

}
