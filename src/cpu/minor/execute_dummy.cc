#include "cpu/minor/execute_dummy.hh"

#include <vector>

#include "base/named.hh"
#include "cpu/minor/buffers.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/pipe_data.hh"
#include "cpu/minor/pipeline.hh"

namespace gem5
{


GEM5_DEPRECATED_NAMESPACE(Minor, minor);
namespace minor
{

  //Constructor
Execute_dummy::Execute_dummy(const std::string &name,
    MinorCPU &cpu_,
    const BaseMinorCPUParams &params,
    Latch<ForwardInstData>::Output inp_,
    Latch<ForwardInstData>::Input out_,
    std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer) :
    Named(name),
    cpu(cpu_),
    inp(inp_),
    out(out_),
    nextStageReserve(next_stage_input_buffer),
    execdummyInfo(params.numThreads),
    threadPriority(0)
{
    if (params.executeInputBufferSize < 1) {
        fatal("%s: executeDummyInputBufferSize must be >= 1 (%d)\n", name,
          params.executeInputBufferSize);
    }

    for (ThreadID tid=0; tid < params.numThreads ; tid++) {
        inputBuffer.push_back(
            InputBuffer<ForwardInstData>(name + ".inputBuffer" +
              std::to_string(tid), "insts", params.executeInputBufferSize));
    }
}

const ForwardInstData * Execute_dummy::getInput(ThreadID tid)
{
    /* Get insts from the inputBuffer to send it to the actual execute stage*/
    if (!inputBuffer[tid].empty()) {
        const ForwardInstData &head = inputBuffer[tid].front();

        return (head.isBubble() ? NULL : &(inputBuffer[tid].front()));
    } else {
        return NULL;
    }
}

void Execute_dummy::popInput(ThreadID tid)
{
    if (!inputBuffer[tid].empty())
          inputBuffer[tid].pop();

}

void Execute_dummy::evaluate()
{
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].setTail(*inp.outputWire);

    ForwardInstData &insts_out = *out.inputWire;

    assert(insts_out.isBubble());

    // Check if for all threadIds can we reserve a space in
    // the buffer in the next stage
    for (ThreadID tid=0; tid < cpu.numThreads; tid++)
       execdummyInfo[tid].blocked = !nextStageReserve[tid].canReserve();

    // deciding which thread to pick and push data to the next stage
    ThreadID tid = getScheduledThread();

    if (tid != InvalidThreadID) {
        // Push the data to the output

        const ForwardInstData *insts_in = getInput(tid);

        insts_out = *insts_in;
        //std::cout << "copied data" ;
        popInput(tid);

        if (!insts_out.isBubble()) {
            cpu.activityRecorder->activity();
            insts_out.threadId = tid;
            nextStageReserve[tid].reserve();
        }

        for (ThreadID i = 0; i < cpu.numThreads; i++)
        {
           if (getInput(i) && nextStageReserve[i].canReserve()) {
              cpu.activityRecorder->activateStage(Pipeline
                ::ExecuteDummyStageId);
               break;
            }
        }


    }
        if (!inp.outputWire->isBubble())
          inputBuffer[inp.outputWire->threadId].pushTail();

}


inline ThreadID
Execute_dummy::getScheduledThread()
{
    /* Select thread via policy. */
    std::vector<ThreadID> priority_list;

    switch (cpu.threadPolicy) {
      case enums::SingleThreaded:
        priority_list.push_back(0);
        break;
      case enums::RoundRobin:
        priority_list = cpu.roundRobinPriority(threadPriority);
        break;
      case enums::Random:
        priority_list = cpu.randomPriority();
        break;
      default:
        panic("Unknown fetch policy");
    }

    for (auto tid : priority_list) {
        if (getInput(tid) && !execdummyInfo[tid].blocked) {
            threadPriority = tid;
            return tid;
        }
    }

   return InvalidThreadID;
}

bool Execute_dummy::isDrained()
{
    for (const auto &buffer : inputBuffer) {
        if (!buffer.empty())
            return false;
    }

    return (*inp.outputWire).isBubble();
}


/*void Execute_dummy::minorTrace() const
{
    std::ostringstream data;

}*/


}


}
