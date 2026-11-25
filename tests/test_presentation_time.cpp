#include "../firmware/common/src/BUSA_Presentation.h"
#include <cassert>
#include <string>
using namespace busa_presentation;
int main(){
    char out[32];
    const struct {uint32_t ms;const char* expected;} cases[]={
        {0,"0ч00мин"},{59999,"0ч00мин"},{180000,"0ч03мин"},
        {3599999,"0ч59мин"},{3600000,"1ч00мин"},{3660000,"1ч01мин"},
        {90000000,"25ч00мин"},{UINT32_MAX,"1193ч02мин"}};
    for(const auto& c:cases){duration(out,sizeof(out),c.ms);assert(std::string(out)==c.expected);}
    assert(std::string(firmwareLabel("technical-PERF4-HW2-NAME1-TIME1"))=="BUSA 1M AI 1v");
    assert(std::string(firmwareLabel("technical-PERF4-HW2-NAME1"))=="BUSA 1M AI 1v");
    assert(std::string(firmwareLabel("technical-PERF4-HW2"))=="technical-PERF4-HW2");
    assert(std::string(firmwareLabel("—"))=="—");
    const char* stamp="26.11.2026 00:19:34";
    assert(std::string(stamp,clockLength(stamp))=="26.11.2026 00:19");
    assert(clockLength("26.11.2026 00:19")==16);
    assert(clockLength("—")==std::strlen("—"));
}
