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

            //Calling the updateCount of the blk
            //This is a new function added in shepherdRP
            // What this does is to check if nvc[set_no][i]!=-1
            // and count[i]=-1 and copies nvc[set_no][i] to count[i]
            // and then increments nvc[set_no][i]
            int scWayNo;
            if (replacementPolicy->getSCFlag(blk->replacementData)) {
               scWayNo = 4-(allocAssoc - blk->getWay());
            }
            replacementPolicy->copyCount(blk->replacementData,
                                          next_value_count,set_no);

            //printf("Shepherd Touch happened");

            if (replacementPolicy->getSCFlag(blk->replacementData)) {
                printf("Touched an SC entry Set_no:%d , Way_no:%d\n",
                blk->getSet(),blk->getWay());

                for (int i=0; i<4; i++) {
                    printf("NVC is:%d",next_value_count[set_no][i]);
                }
                printf("\n");
                //printf("count of the touched set is:%d")
            } else {
                printf("Touched an MC entry Set_no:%d , Way_no:%d\n",
                blk->getSet(),blk->getWay());
            }

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
        //printf("getting victim before set_no");
        int set_no = (static_cast<CacheBlk*>(entries.front()))->getSet();
        printf("--------------Entering findVictim for set no:%d----------\n"
        ,set_no);
        //printf("Set_no in find_victim:%d\n",set_no);
        //printf("got the set_no:%d \n",set_no);
        int sc_head = next_sc_queue[set_no];
        //printf("sc_head for this replacement is %d\n",sc_head);
        // Choose replacement victim from replacement candidates
        CacheBlk* victim = static_cast<CacheBlk*>
                            (replacementPolicy->getVictimSC(
                                entries,sc_head));
        //printf("Exiting victimSC\n");
        // There is only one eviction for this replacement
        evict_blks.push_back(victim);

        printf("----------Exiting findVictim for set no:%d-----------\n"
        , set_no);
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
        printf("------Entered insertBlock set no:%d victim way:%d-------\n",
        blk->getSet(),blk->getWay());
        bool victim_in_sc = false;
        int set_no = blk->getSet();
        const std::vector<ReplaceableEntry*> entries =
                            indexingPolicy->getPossibleEntries(pkt->getAddr());


        // This victim index is pointing to the head of the queue
        // if the replacement's find algorithm is written properly
        // it should set the replacement candidate as either
        // the SC_way correspoding to victim_index
        // or any MC entry
        int victim_index = next_sc_queue[set_no];

        if (blk->isValid()){
            printf("replacing a valid block\n");
          if (replacementPolicy->getSCFlag(blk->replacementData))  {

                // This is the case where victim block is decided
                // as SC block.

                // Insert the new packet there.
                BaseTags::insertBlock(pkt,blk);

                // reset the NVC according to the victim_index to 0
                next_value_count[set_no][victim_index] = 0;// nvc is reset

                 printf("resetting NVC\n");

                // Getting all the entries in the set.
                // update the count[victim_index] of all entries in the
                // cache to e
                // updateCount will set the count[victim_index]
                replacementPolicy->updateCount(entries,victim_index);
                replacementPolicy->resetCount(blk->replacementData,
                                            victim_index);

                next_sc_queue[set_no] = (next_sc_queue[set_no] != 4-1) ?
                next_sc_queue[set_no] + 1 : 0;

          } else { // case where the victim block is in MC
              CacheBlk* sc_head = static_cast<CacheBlk*>
                                  (entries[allocAssoc-4+victim_index]);

                //sc_head->replacementData.isSC = 0;
                //sc_head->replacementData.isMC = 1;
                replacementPolicy->updateSCMCFlags(sc_head->replacementData,
                                                false);
                printf("Attempting to move block\n");
                moveBlock(sc_head,blk);
                // move the scblock to victim's location
                BaseTags::insertBlock(pkt,sc_head);
                printf("Move block successful\n");
                next_value_count[set_no][victim_index] = 0;
                printf("resetting NVC\n");

                // update the count[victim_index] of all entries
                //in the cache to 0
                replacementPolicy->updateCount(entries,victim_index);
                replacementPolicy->resetCount(sc_head->replacementData,
                        victim_index);

                next_sc_queue[set_no] = (next_sc_queue[set_no] != 4-1)
                ? next_sc_queue[set_no] + 1 : 0;
          }
        } else { // there were invalid entries that could be filled.
              printf("Replacing an invalid block\n");
            if (!replacementPolicy->getSCFlag(blk->replacementData)){ //Is MC

                BaseTags::insertBlock(pkt,blk);
                printf("Validity of block at set_no:%d and way_no:%d is %d",
                blk->getSet(),blk->getWay(),blk->isValid());
                // The reset function is modified to set count[i] to -1;
                replacementPolicy->resetCount(blk->replacementData,-1);


            } else { // invalid entry in SC that gets filled.
                printf("Invalid entry in SC filled. isSC:%d",
                replacementPolicy->getSCFlag(blk->replacementData));
                int index = 4 - (allocAssoc - (blk->getWay()));
                printf("Index is:%d way is %d",blk->getWay(),index);

                int set = blk->getSet();
                BaseTags::insertBlock(pkt,blk);
                printf("Validity of block at set_no:%d and way_no:%d is %d",
                blk->getSet(),blk->getWay(),blk->isValid());

                next_value_count[set][index] = 0;
                printf("resetting NVC\n");
                replacementPolicy->updateCount(entries,index);
                replacementPolicy->resetCount(blk->replacementData,index);
                //printf("Blk info:isValid=%d\n",blk->isValid());

                printf("Invalid SC entry filled in set no: %d\n",
                blk->getSet());
                //printf("")
                printf(" Reset nvc for this set no:%d is\n",set);

                for (int i=0;i<4;i++) {
                    printf("%d",next_value_count[set][i]);

               }
            }
        }
          // Increment tag counter
        stats.tagsInUse++;
        printf("------Exited insertBlock after insert set_no:%d------\n"
        ,blk->getSet());

        // Update replacement policy
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
