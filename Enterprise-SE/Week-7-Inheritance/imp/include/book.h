#ifndef BOOK
#define BOOK

#include <publication.h>
class Book : private Publication {
public:
private:
    char* author;
    bool orderCopies();

};
#endif

