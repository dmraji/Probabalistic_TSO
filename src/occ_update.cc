// Update occupancy with new scan data (source)

#include "occ_update.hh"

//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//_//

// Update occupancy hash map with data per pose
void vox_update(spp::sparse_hash_map<ind, opp_data> & opp,
                spp::sparse_hash_map<ind, int> & pocc,
                spp::sparse_hash_map<ind, occ_data> & occ,
                spp::sparse_hash_map<ind, free_unk_data> & freev,
                spp::sparse_hash_map<ind, free_unk_data> & unk,
                int pose_ind
                )
{
  // Construct map iterator
  spp::sparse_hash_map <ind, opp_data> ::iterator it;

  // Iterate through sparse hash map; update intensity based on average intensity of temporary occupancy map, update hits, update definite occupied cells
  for(it = opp.begin(); it != opp.end(); ++it)
  {
    ind cind = {it->first.x,
                it->first.y,
                it->first.z};
    it->second.intensity /= it->second.hits;

    ++occ[cind].hits;
    if(occ[cind].hits > 60) { ++pocc[cind]; }
    occ[cind].intensity = it->second.intensity;
  }

  // Reset temporary occupancy map, but do not clear memory because it will be refilled
  opp.clear();

  // Call to probability updating non-member
  float mean_probability = prob_update(occ,
                                       pose_ind
                                       );

  // Instantiation of bounding box for current occupied pointcloud
  corners bounds(occ,
                 mean_probability
                 );

  // Call to bounding box evening-out member
  bounds.even_out();

  // Update unkown voxels

  // Reset unknown map, but do not clear memory because it will be refilled
  unk.clear();

  // Iterate through bounding box and query to determine whether to add unknown voxel
  for(int x_i = bounds.min_x; x_i <= bounds.max_x; ++x_i)
  {
    for(int y_i = bounds.min_y; y_i <= bounds.max_y; ++y_i)
    {
      for(int z_i = bounds.min_z; z_i <= bounds.max_z; ++z_i)
      {
        // Querying whether voxel has been seen free
        if(freev.count( {x_i, y_i, z_i} ) == 0)
        {
          // Querying whether voxel has been seen occupied
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
  // Z-plane structure biasing map
  spp::sparse_hash_map<int, int> z_vals;
  int max_z_dens = 0;

  float mean_probability = 0.0f;

  // Construct map iterator
  spp::sparse_hash_map <ind, occ_data> ::iterator it;

  // Iterate through occupied map and update z-plane biasing map
  for(it = occ.begin(); it != occ.end(); ++it)
  {
    // Update z-density of occupied cells
    ++z_vals[it->first.z];
    if(z_vals[it->first.z] > max_z_dens) { max_z_dens = z_vals[it->first.z]; }
  }

  // Iterate through occupied map
  for(it = occ.begin(); it != occ.end(); ++it)
  {
    // Update probability with z-biasing
    // it->second.probability = (float)(z_vals[it->first.z] / (float)max_z_dens) *
    //                          (float)it->second.hits / (float)(pose_ind+1);

    // Update probability without z-biasing
    it->second.probability = (float)it->second.hits / (float)(pose_ind+1);

    // Sum to mean_probability
    mean_probability = mean_probability + it->second.probability;
  }

  // Calculate mean_probability based on size of occupied map
  mean_probability = mean_probability / occ.size();

  // Cull occupied space according to occupancy probability
  spp::sparse_hash_map <ind, occ_data> ::iterator it_cull;

  // Arbitrary threshold
  float threshold = 2.0f;

  // Iterate through occupied map to determine voxels to mask
  for(it_cull = occ.begin(); it_cull != occ.end(); ++it_cull)
  {
    ind cind = {it_cull->first.x,
                it_cull->first.y,
                it_cull->first.z};

    // Query whether voxel is above masking probability threshold
    if((occ[cind].probability > (threshold * mean_probability)))
    {
      // If above threshold, mask on
      occ[cind].mask = true;
    }
    else if((occ[cind].probability < (threshold * mean_probability)))
    {
      // If below threshold, mask off
      occ[cind].mask = false;
    }
  }

  // Return the mean probability
  return mean_probability;

}
