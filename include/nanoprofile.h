#ifndef NANOPROFILE_H
#define NANOPROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct np_event
{
    char* Name;
    uint64_t Timestamp;
    uint64_t Duration;
} np_event;

typedef struct np_profile
{
    np_event* EvtBuffer;
    np_event* LastEvt;
    size_t    UsedEvtCount;
    size_t    MaxEvtCount;
    size_t    D;
} np_profile;



#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

static const char* const NP_TRACE_PREAMBLE = "{\n\"traceEvents\": [\n";
static const char* const NP_TRACE_POSTAMBLE = "],\n\"meta_user:\": \"np\",\n\"meta_cpu_count\": \"4\"\n}\n\n";
static struct np_event NP_GlobalEvtStorage[32];
static size_t NP_GlobalLastEvent;


static inline void
NP_InitProfile(np_event* EventBuffer, size_t MaxEventCount, np_profile* ProfileOut)
{
    ProfileOut->EvtBuffer = EventBuffer;
    ProfileOut->MaxEvtCount = MaxEventCount;
    ProfileOut->LastEvt = EventBuffer;
    ProfileOut->UsedEvtCount = 0;
}

static inline uint64_t
NP_Now(void)
{
    LARGE_INTEGER CounterHz;
    QueryPerformanceFrequency(&CounterHz);

    double CountsPerMicrosec = ((double)CounterHz.QuadPart) / 1000000.0;

    LARGE_INTEGER CounterVal;
    QueryPerformanceCounter(&CounterVal);

    double Timestamp = ((double)CounterVal.QuadPart) / CountsPerMicrosec;

    return (uint64_t)Timestamp;
}


static inline void
NP_PushTraceEvent(np_profile* const Profile, const char* const EvtName)
{
    Profile->EvtBuffer[Profile->UsedEvtCount] = { (char*)EvtName, NP_Now(), 0 }; 
    Profile->D = &Profile->EvtBuffer[Profile->UsedEvtCount] - Profile->LastEvt;
    Profile->LastEvt = &(Profile->EvtBuffer[Profile->UsedEvtCount]);
    Profile->UsedEvtCount++;
}

static inline void
NP_PopTraceEvent(np_profile* const Profile)
{
    assert(Profile->LastEvt->Name);
    Profile->LastEvt->Duration = (NP_Now() - Profile->LastEvt->Timestamp);
    printf("SET DUR LAST EVENT %s\n", Profile->LastEvt->Name);
    Profile->LastEvt -= Profile->D;
}


static inline void
NP_WriteJSONTrace(np_profile* Profile, char* Buffer, size_t BuffSize)
{
    FILE* File = fopen("trace.json", "wb"); 
    if (File)
    {
        fprintf(File, NP_TRACE_PREAMBLE);


        for (size_t T = 0; T < Profile->UsedEvtCount; ++T)
        {
            np_event* Current = &Profile->EvtBuffer[T];
            // Write json format for this event
            fprintf(File, "{ \"pid\": 1, \"tid\": 1, \"ts\": %llu, \"dur\": %llu, \"ph\": \"X\", \"name\": \"%s\", \"args\": {} }",
                    Current->Timestamp,
                    Current->Duration,
                    Current->Name);

            if ((Profile->UsedEvtCount - 1) == T) fprintf(File, "\n");
            else fprintf(File, ",\n");

        }

        fprintf(File, NP_TRACE_POSTAMBLE);
        fclose(File);
    }
}




#ifdef __cplusplus
}
#endif

#endif

