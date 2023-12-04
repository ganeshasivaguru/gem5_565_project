/**
 * Copyright (c) 2018-2020 Inria
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

#include "mem/cache/replacement_policies/shepherd_rp.hh"

#include <cassert>
#include <memory>

#include "mem/cache/replacement_policies/base.hh"
#include "params/ShepherdRP.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(ReplacementPolicy, replacement_policy);
namespace replacement_policy
{

Shepherd::Shepherd(const Params &p)
  : Base(p)
{
}

void
Shepherd::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Reset last touch timestamp
    std::static_pointer_cast<ShepherdReplData>(
        replacement_data)->lastTouchTick = Tick(0);

    // When a cache blk is invalidated -- we set all count to INT_MAX
    for (int i=0; i<4; i++) {
        std::static_pointer_cast<ShepherdReplData>(
            replacement_data)->count[i] = INT_MAX;
    }
}

void
Shepherd::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Update last touch timestamp
    std::static_pointer_cast<ShepherdReplData>(
        replacement_data)->lastTouchTick = curTick();
}


void
Shepherd::copyCount(const std::shared_ptr<ReplacementData>& replacement_data,
                                            int next_value_count[4]) const
{
      for (int i=0; i<4; i++) {
        if (std::static_pointer_cast<ShepherdReplData>(
            replacement_data)->count[i] == -1)
        std::static_pointer_cast<ShepherdReplData>(
            replacement_data)->count[i] = next_value_count[i];
    }
}


void
Shepherd::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<ShepherdReplData>(
        replacement_data)->lastTouchTick = curTick();
}

ReplaceableEntry*
Shepherd::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    ReplaceableEntry* victim = candidates[0];
    for (const auto& candidate : candidates) {
        // Update victim entry if necessary


        /*if (std::static_pointer_cast<ShepherdReplData>(
                    candidate->replacementData)->lastTouchTick <
                std::static_pointer_cast<ShepherdReplData>(
                    victim->replacementData)->lastTouchTick) {
            victim = candidate;
        }*/

    }

    return victim;
}

ReplaceableEntry*
Shepherd::getVictimSC(const ReplacementCandidates& candidates, int sc_head)
                                                                const
{
    // There must be atleast one replacement candidate
    assert(candidates.size() > 0);

    int has_e_entry;

    ReplaceableEntry* victim = candidates[0];
    ReplaceableEntry* sc_victim = candidates[0];

    for (const auto& candidate : candidates) {

        if (std::static_pointer_cast<ShepherdReplData>(candidate
        ->replacementData)->count[sc_head] == -1) {
            has_e_entry=1;
            if (std::static_pointer_cast<ShepherdReplData>(
                    candidate->replacementData)->lastTouchTick <
                std::static_pointer_cast<ShepherdReplData>(
                    victim->replacementData)->lastTouchTick) {

                victim = candidate;

            }
        } else if (std::static_pointer_cast<ShepherdReplData>(candidate
           ->replacementData)->count[sc_head] == INT_MAX) {
            return candidate;
            // If any invalid block --> return that as the victim;
        } else {
            if (std::static_pointer_cast<ShepherdReplData>(
                candidate->replacementData)->count[sc_head] >
                std::static_pointer_cast<ShepherdReplData>(
                sc_victim->replacementData)->count[sc_head]) {
                        sc_victim=candidate;
                }
        }
    }
    if (has_e_entry) {
        // USE LRU policy
        return victim;
    } else {
        // USE shepherd policy
        return sc_victim;
    }
}

std::shared_ptr<ReplacementData>
Shepherd::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new ShepherdReplData());
}

void Shepherd::updateCount(const ReplacementCandidates& candidates,
                                int index) const
{
    // This function makes the count[index] to "e(aka -1)" for all
    // the elements of the set in picture.
    for (const auto& candidate : candidates) {
        std::static_pointer_cast<ShepherdReplData>(
            candidate->replacementData)->count[index] = -1;
    }
}

} // namespace replacement_policy
} // namespace gem5
