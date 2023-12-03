/*
 * Copyright (c) 2012-2014 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Definitions of a conventional tag store.
 */

#include "mem/cache/tags/shepherd_tags.hh"

#include <string>

#include "base/intmath.hh"

namespace gem5
{

// This is the constructor
ShepherdTags::ShepherdTags(const Params &p)
    :BaseTags(p), allocAssoc(p.assoc), blks(p.size / p.block_size),
     sequentialAccess(p.sequential_access),
     replacementPolicy(p.replacement_policy),
     numSets(p.size / (p.entry_size * p.assoc))
{
    // There must be a indexing policy
    fatal_if(!p.indexing_policy, "An indexing policy is required");

    // Check parameters
    if (blkSize < 4 || !isPowerOf2(blkSize)) {
        fatal("Block size must be at least 4 and a power of 2");
    }

    //next_value_count = new std::vector<std::array<int,4>>
    // Initialize sc_queue, next_sc_queue and next_value_count
    next_value_count = new int*[numSets];
    for (int i=0, i<numSets; i++) {
            next_value_count[i] = new int[4];
        }

    for (int i=0; i < numSets; i++) {
        for (int j=0; j < 4; j++) {
            //Later this 4 has to be made equal to SC-way size
            //CacheBlk* cblk = new CacheBlk();
            //sc_queue[i][j] = cblk;
            next_value_count[i][j]=-1;
            // Setting next value count to -1 as we increment NVC first
            // before copying to count[4] in replacement data
        }
        next_sc_queue.push_back(0);
    }
}

void
ShepherdTags::tagsInit()
{
    // Initialize all blocks
    for (unsigned blk_index = 0; blk_index < numBlocks; blk_index++) {
        // Locate next cache block
        CacheBlk* blk = &blks[blk_index];


        // Link block to indexing policy
        indexingPolicy->setEntry(blk, blk_index);

        /* GVS*/
        /* Trying to allocate sc_queue to the last 4 blocks in a set*/
        const std::lldiv_t result = std::div((long long)blk_index, allocAssoc);
        const uint32_t set = result.quot;
        const uint32_t way = result.rem;


        // Associate a data chunk to the block
        blk->data = &dataBlks[blkSize*blk_index];

        // Associate a replacement data entry to the block
        blk->replacementData = replacementPolicy->instantiateEntry();
        if (way >= allocAssoc - 4) { // IS SC entries
            //sc_queue[set][way - (allocAssoc - 4)] = blk;
            blk->replacementData->isSC = true;
            blk->replacementData->isMC = false;
            if (!blk->isValid()) // INVALID block's count set to INT_MAX
                                // so that it gets evicted first.
                blk->replacementData->count[way-(allocAssoc-4)] = INT_MAX;

        } else { // IS MC entries
            blk->replacementData->isSC = false;
            blk->replacementData->isMC = true;
            if (!blk->isValid()) {
                for (int i=0; i<4; i++) {
                    blk->replacementData->count[i] = INT_MAX:
                }
            }

        }
    }
}

void
ShepherdTags::invalidate(CacheBlk *blk)
{
    BaseTags::invalidate(blk);

    // Decrease the number of tags in use
    stats.tagsInUse--;

    // Invalidate replacement data
    replacementPolicy->invalidate(blk->replacementData);
}

void
ShepherdTags::moveBlock(CacheBlk *src_blk, CacheBlk *dest_blk)
{
    BaseTags::moveBlock(src_blk, dest_blk);

    // Since the blocks were using different replacement data pointers,
    // we must touch the replacement data of the new entry, and invalidate
    // the one that is being moved.
    replacementPolicy->invalidate(src_blk->replacementData);
    replacementPolicy->reset(dest_blk->replacementData);
}

} // namespace gem5
