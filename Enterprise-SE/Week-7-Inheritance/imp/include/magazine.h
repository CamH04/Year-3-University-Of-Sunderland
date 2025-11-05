#ifndef MAGAZINE
#define MAGAZINE

#include <publication.h>
class Magazine : private Publication {
public:
private:
    int orderQty;
    int currIssue();
    void updateQty();
    void reciveNewIssue();

};
#endif


