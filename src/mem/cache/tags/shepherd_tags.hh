/*
 * Copyright (c) 2012-2014,2017 ARM Limited
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
 * Declaration of a base set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_SHEPHERD_TAGS_HH__
#define __MEM_CACHE_TAGS_SHEPHERD_TAGS_HH__

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/packet.hh"
#include "params/ShepherdTags.hh"

namespace gem5
{

/**
 * A basic cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The ShepherdTags placement policy divides the cache into s sets of w
 * cache lines (ways). In addition to this, there are addition members to keep
 * track of the various functions needed by the Shepherd Cache.
 */
class ShepherdTags : public BaseTags
{
  protected:
    /** The allocatable associativity of the cache (alloc mask). */
    unsigned allocAssoc;

    /** Number of sets*/
    unsigned numSets;

    /** The cache blocks. */
    std::vector<CacheBlk> blks;

    /** Whether tags and data are accessed sequentially. */
    const bool sequentialAccess;

    /** Replacement policy */
    replacement_policy::Base *replacementPolicy;

  public:

    //CacheBlk*** sc_queue;
    //Vector of arrays pointing to sc entries in each set.
    // The vector will have NumSets number of entries.

    std::vector<int> next_sc_queue;
    // This will point to the next SC_queue entry that will be replaced.
    // The vector will have NumSets number of entries.

    int** next_value_count;
    // Creating a vector of next_value_count where each entry

  public:
    /** Convenience typedef. */
     typedef ShepherdTagsParams Params;

    /**
     * Construct and initialize this tag store.
     */
    ShepherdTags(const Params &p);

    /**
     * Destructor
     */
    virtual ~ShepherdTags() {};
    //delete next_value_count;
    //delete next_sc_queue;

    /**
     * Initialize blocks as CacheBlk instances.
     */
    void tagsInit() override;

    /**
     * This function updates the tags when a block is invalidated. It also
     * updates the replacement data.
     *
     * @param blk The block to invalidate.
     */
    void invalidate(CacheBlk *blk) override;

    /**
     * Access block and update replacement data. May not succeed, in which case
     * nullptr is returned. This has all the implications of a cache access and
     * should only be used as such. Returns the tag lookup latency as a side
     * effect.
     *
     * @param pkt The packet holding the address to find.
     * @param lat The latency of the tag lookup.
     * @return Pointer to the cache block if found.
     */
    CacheBlk* accessBlock(const PacketPtr pkt, Cycles &lat) override
    //GVS: Edit this based on the changes in the document
    {
        CacheBlk *blk = findBlock(pkt->getAddr(), pkt->isSecure());

        // Access all tags in parallel, hence one in each way.  The data side
        // either accesses all blocks in parallel, or one block sequentially on
        // a hit.  Sequential access with a miss doesn't access data.
        stats.tagAccesses += allocAssoc;
        if (sequentialAccess) {
            if (blk != nullptr) {
                stats.dataAccesses += 1;
            }
        } else {
            stats.dataAccesses += allocAssoc;
        }

        // If a cache hit
        if (blk != nullptr) {
            // Update number of references to accessed block
            blk->increaseRefCount();

            // Update replacement data of accessed block
            replacementPolicy->touch(blk->replacementData, pkt);

            /* GVS*/
            /* Updating the nvc count and then copying it to the count*/
            // getting the blk's set number
            uint64_t set_no = blk->getSet();
            const std::vector<ReplaceableEntry*> entries =
            indexingPolicy->getPossibleEntries(pkt->getAddr());



            for (int i=0; i<4; i++) {
                CacheBlk* sc_entry = static_cast<CacheBlk*>
                                      (entries[allocAssoc-4+i]);
                if (sc_entry->isValid()) {
                    next_value_count[set_no][i]++;
                }
            }
            //Calling the updateCount of the blk
            //This is a new function added in shepherdRP
            //We will send the blk->replacementData which contains the
            // count[4] and also nvc corresponding to that set
            // Calling this after the for loop -- as for loop deciding on
            // whether to update or not for each SC-way nvc.
            replacementPolicy->copyCount(blk->replacementData,
                                          next_value_count[set_no]);


        }

        // The tag lookup latency is the same for a hit or a miss
        lat = lookupLatency;

        return blk;
    }

    /**
     * Find replacement victim based on address. The list of evicted blocks
     * only contains the victim.
     *
     * @param addr Address to find a victim for.
     * @param is_secure True if the target memory space is secure.
     * @param size Size, in bits, of new block to allocate.
     * @param evict_blks Cache blocks to be evicted.
     * @return Cache block to be replaced.
     */
    CacheBlk* findVictim(Addr addr, const bool is_secure,
                         const std::size_t size,
                         std::vector<CacheBlk*>& evict_blks) override
    //GVS: edit this based on the changes in the document
    {
        // Get possible entries to be victimized
        const std::vector<ReplaceableEntry*> entries =
            indexingPolicy->getPossibleEntries(addr);

        // Logic to get the sc_head;
        int set_no = (static_cast<CacheBlk*>(entries.front()))->getSet();
        int sc_head = next_sc_queue[set_no];

        // Choose replacement victim from replacement candidates
        CacheBlk* victim = static_cast<CacheBlk*>
                            (replacementPolicy->getVictimSC(
                                entries,sc_head));

        // There is only one eviction for this replacement
        evict_blks.push_back(victim);

        return victim;
    }

    /**
     * Insert the new block into the cache and update replacement data.
     *
     * @param pkt Packet holding the address to update
     * @param blk The block to update.
     */
    void insertBlock(const PacketPtr pkt, CacheBlk *blk) override
    // GVS: edit this based on the changes in the document
    {
        // Insert block
        //BaseTags::insertBlock(pkt, blk);

        bool victim_in_sc = false;
        int set_no = blk->getSet();
        const std::vector<ReplaceableEntry*> entries =
                            indexingPolicy->getPossibleEntries(pkt->getAddr());


        int victim_index = next_sc_queue[set_no];
        if (blk->isValid()){
          if (blk->replacementData.isSC)  {
              BaseTags::insertBlock(pkt,blk);
              next_value_count[set_no][victim_index] = -1;
              // Getting all the entries in the set.
              // update the count[victim_index] of all entries in the
              // cache to e
              replacementPolicy->updateCount(entries,victim_index);
              //In updateCount set count[4] to -1.
                next_sc_queue[set_no] = (next_sc_queue[set_no] != 4-1) ?
                next_sc_queue[set_no] + 1 : 0;
          } else { // case where the victim block is in MC
              CacheBlk* sc_head = static_cast<CacheBlk*>
                                  (entries[allocAssoc-4+victim_index]);
              sc_head->replacementData.isSC = 0;
              sc_head->replacementData.isMC = 1;
              moveBlock(sc_head,blk); // move the scblock to victim's location
              BaseTags::insertBlock(pkt,sc_head);
              next_value_count[set_no][victim_index] = -1;
              // update the count[victim_index] of all entries
              //in the cache to 0
              replacementPolicy->updateCount(entries,victim_index);
                next_sc_queue[set_no] = (next_sc_queue[set_no] != 4-1)
                ? next_sc_queue[set_no] + 1 : 0;
          }
        } else { // there were invalid entries that could be filled.
            if (blk->replacementData.isMC){
                for (int i=0; i<4; i++)  {
                    blk->replacementData.count[i] = -1;
                }
            } else {
               int index = allocAssoc - (blk->getWay());

               replacementPolicy->updateCount(entries,index);

               for (int i=0;i<4;i++) {
                    if (i==index){
                        blk->replacementData.count[i] = -1
                    } else {
                        blk->replacementData.count[i] = 0;
                    }
               }
            }
            // Insert the blk;
            BaseTags::insertBlock(pkt,blk);
        }
          // Increment tag counter
        stats.tagsInUse++;

        // Update replacement policy
        replacementPolicy->reset(blk->replacementData, pkt);
    }

    void moveBlock(CacheBlk *src_blk, CacheBlk *dest_blk) override;

    /**
     * Limit the allocation for the cache ways.
     * @param ways The maximum number of ways available for replacement.
     */
    virtual void setWayAllocationMax(int ways) override
    {
        fatal_if(ways < 1, "Allocation limit must be greater than zero");
        allocAssoc = ways;
    }

    /**
     * Get the way allocation mask limit.
     * @return The maximum number of ways available for replacement.
     */
    virtual int getWayAllocationMax() const override
    {
        return allocAssoc;
    }

    /**
     * Regenerate the block address from the tag and indexing location.
     *
     * @param block The block.
     * @return the block address.
     */
    Addr regenerateBlkAddr(const CacheBlk* blk) const override
    {
        return indexingPolicy->regenerateAddr(blk->getTag(), blk);
    }

    void forEachBlk(std::function<void(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            visitor(blk);
        }
    }

    bool anyBlk(std::function<bool(CacheBlk &)> visitor) override {
        for (CacheBlk& blk : blks) {
            if (visitor(blk)) {
                return true;
            }
        }
        return false;
    }
};

} // namespace gem5

#endif //__MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
