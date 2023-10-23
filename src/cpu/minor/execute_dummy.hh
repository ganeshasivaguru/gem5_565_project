#ifndef __CPU_MINOR_EDUMMY_HH__
#define __CPU_MINOR_EDUMMY_HH__

#include <vector>

#include <base/named.hh>
#include <base/types.hh>

#include "cpu/minor/buffers.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/pipe_data.hh"

// Not adding func_unit, lsq, scoreboard as this stage is dummy : GVS

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Minor, minor);
namespace minor
{


  /** Dummy execute stage with a attempt to observe more stalls
  */

  class Execute_dummy : public Named
  {
    protected:

      MinorCPU &cpu; //GVS: Rethink if this is needed?
      /** Input port carrying instructions from Decode*/
      Latch<ForwardInstData>::Output inp;

      /* Port carrying instructions to the actual Execute stage*/
      Latch<ForwardInstData>::Input out;

      /** Pointer back to the containing CPU*/

      std::vector<InputBuffer<ForwardInstData>> &nextStageReserve;

    public:
      std::vector<InputBuffer<ForwardInstData>> inputBuffer;

    protected:

       struct ExecdummyInfo
       {
          ExecdummyInfo() {}

          ExecdummyInfo(const ExecdummyInfo& other) :
              blocked(other.blocked)
          { }

          bool blocked = false;

       };

        std::vector<ExecdummyInfo> execdummyInfo;
        ThreadID threadPriority;

    protected:
      const ForwardInstData *getInput(ThreadID tid);

      void popInput(ThreadID tid);


      /** Use the current threading policy to determine the next thread to
      *  push data from. */
      ThreadID getScheduledThread();


    public:
      Execute_dummy(const std::string &name,
          MinorCPU &cpu_,
          const BaseMinorCPUParams &params,
          Latch<ForwardInstData>::Output inp_,
          Latch<ForwardInstData>::Input out_,
          std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer);

        // Do we add a destructor or not ??

    public:
       void evaluate();

       //void minorTrace() const;

       bool isDrained();
  };




} //namespace minor
} // namespace gem5

#endif /* __CPU_MINOR_EDUMMY_HH__ */
