#pragma once

// macro utilities, based on:
// http://saadahmad.ca/cc-preprocessor-metaprogramming-basic-pattern-matching-macros-and-conditionals/
// https://github.com/pfultz2/Cloak/wiki/C-Preprocessor-tricks,-tips,-and-idioms#deferred-expression

#define M_MAX_ARITHMETIC 126 // Max number to be used in arithmetic

#define M_EMPTY()
#define M_DEFER(...) __VA_ARGS__ M_EMPTY()
#define M_DEFER2(...) __VA_ARGS__ M_DEFER(M_EMPTY)()
#define M_DEFER3(...) __VA_ARGS__ M_DEFER2(M_EMPTY)()
#define M_DEFER4(...) __VA_ARGS__ M_DEFER3(M_EMPTY)()
#define M_DEFER5(...) __VA_ARGS__ M_DEFER4(M_EMPTY)()

#define M_EVAL_1(...) __VA_ARGS__
#define M_EVAL_2(...) M_EVAL_1(M_EVAL_1(__VA_ARGS__))
#define M_EVAL_3(...) M_EVAL_2(M_EVAL_2(__VA_ARGS__))
#define M_EVAL_4(...) M_EVAL_3(M_EVAL_3(__VA_ARGS__))
#define M_EVAL_5(...) M_EVAL_4(M_EVAL_4(__VA_ARGS__))
#define M_EVAL_6(...) M_EVAL_5(M_EVAL_5(__VA_ARGS__))
#define M_EVAL_7(...) M_EVAL_6(M_EVAL_6(__VA_ARGS__))
#define M_EVAL_8(...) M_EVAL_7(M_EVAL_7(__VA_ARGS__))
#define M_EVAL(...) M_EVAL_8(__VA_ARGS__)

#define M_EAT(...)
#define M_EXPAND(...) __VA_ARGS__
#define M_OBSTRUCT(...) __VA_ARGS__ M_DEFER(M_EMPTY)()

#define M_ENCLOSE_EXPAND(...) EXPANDED, ENCLOSED, (__VA_ARGS__) ) M_EAT (
#define M_GET_CAT_EXP(a, b) (a, M_ENCLOSE_EXPAND b, DEFAULT, b)

#define M_CAT_WITH_ENCLOSED(a, b) a b
#define M_CAT_WITH_DEFAULT(a, b) a##b
#define M_CAT_WITH(a, _, f, b) M_CAT_WITH_##f(a, b)

#define M_EVAL_CAT_WITH(...) M_CAT_WITH __VA_ARGS__
#define M_CAT(a, b) M_EVAL_CAT_WITH(M_GET_CAT_EXP(a, b))

#define M_HEAD(x, ...) x
#define M_HEAD1(x, y, ...) y
#define M_HEAD2(x, y, z, ...) z

#define M_TAIL(x, ...) __VA_ARGS__
#define M_TAIL1(x, y, ...) __VA_ARGS__
#define M_TAIL2(x, y, z, ...) __VA_ARGS__

#define _M_STRINGIFY(x) #x
#define M_STRINGIFY(x) _M_STRINGIFY(x)

#define M_COMMA ,

#define M_LEN(array) (sizeof(array) / sizeof(array[0]))

// for range stuff:
#define M_EXPAND_TEST_EXISTS(...) EXPANDED, EXISTS(__VA_ARGS__) ) M_EAT (
#define M_GET_TEST_EXISTS_RESULT(x) (CAT(EXPAND_TEST_, x), DOESNT_EXIST)

#define M_GET_TEST_EXIST_VALUE_(expansion, existValue) existValue
#define M_GET_TEST_EXIST_VALUE(x) M_GET_TEST_EXIST_VALUE_ x

#define M_TEST_EXISTS(x) M_GET_TEST_EXIST_VALUE(M_GET_TEST_EXISTS_RESULT(x))

#define M_DOES_VALUE_EXIST_EXISTS(...) 1
#define M_DOES_VALUE_EXIST_DOESNT_EXIST 0
#define M_DOES_VALUE_EXIST(x) M_CAT(M_DOES_VALUE_EXIST_, x)

#define M_EXTRACT_VALUE_EXISTS(...) __VA_ARGS__
#define M_EXTRACT_VALUE(value) M_CAT(M_EXTRACT_VALUE_, value)

#define M_TRY_EXTRACT_EXISTS(value, ...)                                       \
  M_IF(M_DOES_VALUE_EXIST(M_TEST_EXISTS(value)))                               \
  (M_EXTRACT_VALUE(value), __VA_ARGS__)

#define M_NOT_0 EXISTS(1)
#define M_NOT(x) M_TRY_EXTRACT_EXISTS(CAT(NOT_, x), 0)

#define M_IF_1(true, ...) true
#define M_IF_0(true, ...) __VA_ARGS__
#define M_IF(value) M_CAT(M_IF_, value)

#define M_PRIMITIVE_CAT(a, ...) a##__VA_ARGS__
#define M_BITAND(x) M_PRIMITIVE_CAT(M_BITAND_, x)
#define M_BITAND_0(y) 0
#define M_BITAND_1(y) y

#define M_COMPL(b) M_PRIMITIVE_CAT(M_COMPL_, b)
#define M_COMPL_0 1
#define M_COMPL_1 0

#define M_CHECK_N(x, n, ...) n
#define M_CHECK(...) M_CHECK_N(__VA_ARGS__, 0, )
#define M_PROBE(x) x, 1,

#define M_IS_PAREN(x) M_CHECK(M_IS_PAREN_PROBE x)
#define M_IS_PAREN_PROBE(...) M_PROBE(~)

#define M_PRIMITIVE_COMPARE(x, y) M_IS_PAREN(M_COMPARE_##x(M_COMPARE_##y)(()))

#define M_IS_COMPARABLE(x) M_IS_PAREN(M_CAT(M_COMPARE_, x)(()))

#define M_NOT_EQUAL(x, y)                                                      \
  M_IF(M_BITAND(M_IS_COMPARABLE(x))(M_IS_COMPARABLE(y)))                       \
  (M_PRIMITIVE_COMPARE, 1 M_EAT)(x, y)

// M_RANGE example:
// #define PR(i, _) printf("iteration: %d\n", i);
//_RANGE(PR, 0, 8))
#define M_RANGE(macro, start, end, ...)                                        \
  M_IF(M_NOT_EQUAL(start, end))                                                \
  (M_OBSTRUCT(M_RANGE_INDIRECT)()(macro, start, M_DEC(end),                    \
                                  __VA_ARGS__)M_OBSTRUCT(macro)(M_DEC(end),    \
                                                                __VA_ARGS__))
#define M_RANGE_INDIRECT() M_RANGE

#define M_RANGE_WITH_COMMA(macro, start, end, ...)                             \
  M_IF(M_NOT_EQUAL(start, end))                                                \
  (M_OBSTRUCT(M_RANGE_WITH_COMMA_INDIRECT)()(                                  \
      macro, start, M_DEC(end),                                                \
      __VA_ARGS__)M_IF(M_NOT_EQUAL(start, M_DEC(end)))(M_EXPAND(, ))           \
       M_OBSTRUCT(macro)(M_DEC(end), __VA_ARGS__))
#define M_RANGE_WITH_COMMA_INDIRECT() M_RANGE_WITH_COMMA

#define M_COMPARE_0(x) x
#define M_COMPARE_1(x) x
#define M_COMPARE_2(x) x
#define M_COMPARE_3(x) x
#define M_COMPARE_4(x) x
#define M_COMPARE_5(x) x
#define M_COMPARE_6(x) x
#define M_COMPARE_7(x) x
#define M_COMPARE_8(x) x
#define M_COMPARE_9(x) x
#define M_COMPARE_10(x) x
#define M_COMPARE_11(x) x
#define M_COMPARE_12(x) x
#define M_COMPARE_13(x) x
#define M_COMPARE_14(x) x
#define M_COMPARE_15(x) x
#define M_COMPARE_16(x) x
#define M_COMPARE_17(x) x
#define M_COMPARE_18(x) x
#define M_COMPARE_19(x) x
#define M_COMPARE_20(x) x
#define M_COMPARE_21(x) x
#define M_COMPARE_22(x) x
#define M_COMPARE_23(x) x
#define M_COMPARE_24(x) x
#define M_COMPARE_25(x) x
#define M_COMPARE_26(x) x
#define M_COMPARE_27(x) x
#define M_COMPARE_28(x) x
#define M_COMPARE_29(x) x
#define M_COMPARE_30(x) x
#define M_COMPARE_31(x) x
#define M_COMPARE_32(x) x
#define M_COMPARE_33(x) x
#define M_COMPARE_34(x) x
#define M_COMPARE_35(x) x
#define M_COMPARE_36(x) x
#define M_COMPARE_37(x) x
#define M_COMPARE_38(x) x
#define M_COMPARE_39(x) x
#define M_COMPARE_40(x) x
#define M_COMPARE_41(x) x
#define M_COMPARE_42(x) x
#define M_COMPARE_43(x) x
#define M_COMPARE_44(x) x
#define M_COMPARE_45(x) x
#define M_COMPARE_46(x) x
#define M_COMPARE_47(x) x
#define M_COMPARE_48(x) x
#define M_COMPARE_49(x) x
#define M_COMPARE_50(x) x
#define M_COMPARE_51(x) x
#define M_COMPARE_52(x) x
#define M_COMPARE_53(x) x
#define M_COMPARE_54(x) x
#define M_COMPARE_55(x) x
#define M_COMPARE_56(x) x
#define M_COMPARE_57(x) x
#define M_COMPARE_58(x) x
#define M_COMPARE_59(x) x
#define M_COMPARE_60(x) x
#define M_COMPARE_61(x) x
#define M_COMPARE_62(x) x
#define M_COMPARE_63(x) x
#define M_COMPARE_64(x) x
#define M_COMPARE_65(x) x
#define M_COMPARE_66(x) x
#define M_COMPARE_67(x) x
#define M_COMPARE_68(x) x
#define M_COMPARE_69(x) x
#define M_COMPARE_70(x) x
#define M_COMPARE_71(x) x
#define M_COMPARE_72(x) x
#define M_COMPARE_73(x) x
#define M_COMPARE_74(x) x
#define M_COMPARE_75(x) x
#define M_COMPARE_76(x) x
#define M_COMPARE_77(x) x
#define M_COMPARE_78(x) x
#define M_COMPARE_79(x) x
#define M_COMPARE_80(x) x
#define M_COMPARE_81(x) x
#define M_COMPARE_82(x) x
#define M_COMPARE_83(x) x
#define M_COMPARE_84(x) x
#define M_COMPARE_85(x) x
#define M_COMPARE_86(x) x
#define M_COMPARE_87(x) x
#define M_COMPARE_88(x) x
#define M_COMPARE_89(x) x
#define M_COMPARE_90(x) x
#define M_COMPARE_91(x) x
#define M_COMPARE_92(x) x
#define M_COMPARE_93(x) x
#define M_COMPARE_94(x) x
#define M_COMPARE_95(x) x
#define M_COMPARE_96(x) x
#define M_COMPARE_97(x) x
#define M_COMPARE_98(x) x
#define M_COMPARE_99(x) x
#define M_COMPARE_100(x) x
#define M_COMPARE_101(x) x
#define M_COMPARE_102(x) x
#define M_COMPARE_103(x) x
#define M_COMPARE_104(x) x
#define M_COMPARE_105(x) x
#define M_COMPARE_106(x) x
#define M_COMPARE_107(x) x
#define M_COMPARE_108(x) x
#define M_COMPARE_109(x) x
#define M_COMPARE_110(x) x
#define M_COMPARE_111(x) x
#define M_COMPARE_112(x) x
#define M_COMPARE_113(x) x
#define M_COMPARE_114(x) x
#define M_COMPARE_115(x) x
#define M_COMPARE_116(x) x
#define M_COMPARE_117(x) x
#define M_COMPARE_118(x) x
#define M_COMPARE_119(x) x
#define M_COMPARE_120(x) x
#define M_COMPARE_121(x) x
#define M_COMPARE_122(x) x
#define M_COMPARE_123(x) x
#define M_COMPARE_124(x) x
#define M_COMPARE_125(x) x
#define M_COMPARE_126(x) x
#define M_COMPARE_127(x) x

// #define DEC_0 127
#define M_DEC_1 0
#define M_DEC_2 1
#define M_DEC_3 2
#define M_DEC_4 3
#define M_DEC_5 4
#define M_DEC_6 5
#define M_DEC_7 6
#define M_DEC_8 7
#define M_DEC_9 8
#define M_DEC_10 9
#define M_DEC_11 10
#define M_DEC_12 11
#define M_DEC_13 12
#define M_DEC_14 13
#define M_DEC_15 14
#define M_DEC_16 15
#define M_DEC_17 16
#define M_DEC_18 17
#define M_DEC_19 18
#define M_DEC_20 19
#define M_DEC_21 20
#define M_DEC_22 21
#define M_DEC_23 22
#define M_DEC_24 23
#define M_DEC_25 24
#define M_DEC_26 25
#define M_DEC_27 26
#define M_DEC_28 27
#define M_DEC_29 28
#define M_DEC_30 29
#define M_DEC_31 30
#define M_DEC_32 31
#define M_DEC_33 32
#define M_DEC_34 33
#define M_DEC_35 34
#define M_DEC_36 35
#define M_DEC_37 36
#define M_DEC_38 37
#define M_DEC_39 38
#define M_DEC_40 39
#define M_DEC_41 40
#define M_DEC_42 41
#define M_DEC_43 42
#define M_DEC_44 43
#define M_DEC_45 44
#define M_DEC_46 45
#define M_DEC_47 46
#define M_DEC_48 47
#define M_DEC_49 48
#define M_DEC_50 49
#define M_DEC_51 50
#define M_DEC_52 51
#define M_DEC_53 52
#define M_DEC_54 53
#define M_DEC_55 54
#define M_DEC_56 55
#define M_DEC_57 56
#define M_DEC_58 57
#define M_DEC_59 58
#define M_DEC_60 59
#define M_DEC_61 60
#define M_DEC_62 61
#define M_DEC_63 62
#define M_DEC_64 63
#define M_DEC_65 64
#define M_DEC_66 65
#define M_DEC_67 66
#define M_DEC_68 67
#define M_DEC_69 68
#define M_DEC_70 69
#define M_DEC_71 70
#define M_DEC_72 71
#define M_DEC_73 72
#define M_DEC_74 73
#define M_DEC_75 74
#define M_DEC_76 75
#define M_DEC_77 76
#define M_DEC_78 77
#define M_DEC_79 78
#define M_DEC_80 79
#define M_DEC_81 80
#define M_DEC_82 81
#define M_DEC_83 82
#define M_DEC_84 83
#define M_DEC_85 84
#define M_DEC_86 85
#define M_DEC_87 86
#define M_DEC_88 87
#define M_DEC_89 88
#define M_DEC_90 89
#define M_DEC_91 90
#define M_DEC_92 91
#define M_DEC_93 92
#define M_DEC_94 93
#define M_DEC_95 94
#define M_DEC_96 95
#define M_DEC_97 96
#define M_DEC_98 97
#define M_DEC_99 98
#define M_DEC_100 99
#define M_DEC_101 100
#define M_DEC_102 101
#define M_DEC_103 102
#define M_DEC_104 103
#define M_DEC_105 104
#define M_DEC_106 105
#define M_DEC_107 106
#define M_DEC_108 107
#define M_DEC_109 108
#define M_DEC_110 109
#define M_DEC_111 110
#define M_DEC_112 111
#define M_DEC_113 112
#define M_DEC_114 113
#define M_DEC_115 114
#define M_DEC_116 115
#define M_DEC_117 116
#define M_DEC_118 117
#define M_DEC_119 118
#define M_DEC_120 119
#define M_DEC_121 120
#define M_DEC_122 121
#define M_DEC_123 122
#define M_DEC_124 123
#define M_DEC_125 124
#define M_DEC_126 125
#define M_DEC_127 126
#define M_DEC(n) M_CAT(M_DEC_, n)

#define M_INC_0 1
#define M_INC_1 2
#define M_INC_2 3
#define M_INC_3 4
#define M_INC_4 5
#define M_INC_5 6
#define M_INC_6 7
#define M_INC_7 8
#define M_INC_8 9
#define M_INC_9 10
#define M_INC_10 11
#define M_INC_11 12
#define M_INC_12 13
#define M_INC_13 14
#define M_INC_14 15
#define M_INC_15 16
#define M_INC_16 17
#define M_INC_17 18
#define M_INC_18 19
#define M_INC_19 20
#define M_INC_20 21
#define M_INC_21 22
#define M_INC_22 23
#define M_INC_23 24
#define M_INC_24 25
#define M_INC_25 26
#define M_INC_26 27
#define M_INC_27 28
#define M_INC_28 29
#define M_INC_29 30
#define M_INC_30 31
#define M_INC_31 32
#define M_INC_32 33
#define M_INC_33 34
#define M_INC_34 35
#define M_INC_35 36
#define M_INC_36 37
#define M_INC_37 38
#define M_INC_38 39
#define M_INC_39 40
#define M_INC_40 41
#define M_INC_41 42
#define M_INC_42 43
#define M_INC_43 44
#define M_INC_44 45
#define M_INC_45 46
#define M_INC_46 47
#define M_INC_47 48
#define M_INC_48 49
#define M_INC_49 50
#define M_INC_50 51
#define M_INC_51 52
#define M_INC_52 53
#define M_INC_53 54
#define M_INC_54 55
#define M_INC_55 56
#define M_INC_56 57
#define M_INC_57 58
#define M_INC_58 59
#define M_INC_59 60
#define M_INC_60 61
#define M_INC_61 62
#define M_INC_62 63
#define M_INC_63 64
#define M_INC_64 65
#define M_INC_65 66
#define M_INC_66 67
#define M_INC_67 68
#define M_INC_68 69
#define M_INC_69 70
#define M_INC_70 71
#define M_INC_71 72
#define M_INC_72 73
#define M_INC_73 74
#define M_INC_74 75
#define M_INC_75 76
#define M_INC_76 77
#define M_INC_77 78
#define M_INC_78 79
#define M_INC_79 80
#define M_INC_80 81
#define M_INC_81 82
#define M_INC_82 83
#define M_INC_83 84
#define M_INC_84 85
#define M_INC_85 86
#define M_INC_86 87
#define M_INC_87 88
#define M_INC_88 89
#define M_INC_89 90
#define M_INC_90 91
#define M_INC_91 92
#define M_INC_92 93
#define M_INC_93 94
#define M_INC_94 95
#define M_INC_95 96
#define M_INC_96 97
#define M_INC_97 98
#define M_INC_98 99
#define M_INC_99 100
#define M_INC_100 101
#define M_INC_101 102
#define M_INC_102 103
#define M_INC_103 104
#define M_INC_104 105
#define M_INC_105 106
#define M_INC_106 107
#define M_INC_107 108
#define M_INC_108 109
#define M_INC_109 110
#define M_INC_110 111
#define M_INC_111 112
#define M_INC_112 113
#define M_INC_113 114
#define M_INC_114 115
#define M_INC_115 116
#define M_INC_116 117
#define M_INC_117 118
#define M_INC_118 119
#define M_INC_119 120
#define M_INC_120 121
#define M_INC_121 122
#define M_INC_122 123
#define M_INC_123 124
#define M_INC_124 125
#define M_INC_125 126
#define M_INC_126 127
#define M_INC_127 0
#define M_INC(n) M_CAT(M_INC_, n)
