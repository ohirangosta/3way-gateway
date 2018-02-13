#define UD -1
#define DEFAULT_INTERFACE 1
#define INTERFACE 2
#define CAN0 3
#define CAN1 4
#define CAN2 5
#define PASS 6
#define DROP 7
#define CAN_ID 8
#define CAN_ID_NUM 9
#define ENDOFFILE 0
#define SPACE 10
#define NL 11

// parser parts define
struct AbstSyntaxTree{
    int PassOrDrop; //pass:1 or drop:0
    char can_id[4]; //can id
    char apply_rule_interface[5]; //interface A pass B C A:apply_rule_interface, BorC:another_interface
    char another_interface1[5];
    char another_interface2[5];
    struct AbstSyntaxTree *next_rule; //next rule
};
