/****************************************************************************
** Meta object code from reading C++ file 'CurveConfig.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rqt_multiplot_plugin/include/rqt_multiplot/CurveConfig.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CurveConfig.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rqt_multiplot__CurveConfig_t {
    QByteArrayData data[11];
    char stringdata0[165];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_multiplot__CurveConfig_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_multiplot__CurveConfig_t qt_meta_stringdata_rqt_multiplot__CurveConfig = {
    {
QT_MOC_LITERAL(0, 0, 26), // "rqt_multiplot::CurveConfig"
QT_MOC_LITERAL(1, 27, 12), // "titleChanged"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 5), // "title"
QT_MOC_LITERAL(4, 47, 26), // "subscriberQueueSizeChanged"
QT_MOC_LITERAL(5, 74, 6), // "size_t"
QT_MOC_LITERAL(6, 81, 9), // "queueSize"
QT_MOC_LITERAL(7, 91, 17), // "axisConfigChanged"
QT_MOC_LITERAL(8, 109, 18), // "colorConfigChanged"
QT_MOC_LITERAL(9, 128, 18), // "styleConfigChanged"
QT_MOC_LITERAL(10, 147, 17) // "dataConfigChanged"

    },
    "rqt_multiplot::CurveConfig\0titleChanged\0"
    "\0title\0subscriberQueueSizeChanged\0"
    "size_t\0queueSize\0axisConfigChanged\0"
    "colorConfigChanged\0styleConfigChanged\0"
    "dataConfigChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_multiplot__CurveConfig[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,
       4,    1,   47,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   50,    2, 0x08 /* Private */,
       8,    0,   51,    2, 0x08 /* Private */,
       9,    0,   52,    2, 0x08 /* Private */,
      10,    0,   53,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, 0x80000000 | 5,    6,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rqt_multiplot::CurveConfig::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CurveConfig *_t = static_cast<CurveConfig *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->titleChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->subscriberQueueSizeChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 2: _t->axisConfigChanged(); break;
        case 3: _t->colorConfigChanged(); break;
        case 4: _t->styleConfigChanged(); break;
        case 5: _t->dataConfigChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CurveConfig::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CurveConfig::titleChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CurveConfig::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CurveConfig::subscriberQueueSizeChanged)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rqt_multiplot::CurveConfig::staticMetaObject = {
    { &Config::staticMetaObject, qt_meta_stringdata_rqt_multiplot__CurveConfig.data,
      qt_meta_data_rqt_multiplot__CurveConfig,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *rqt_multiplot::CurveConfig::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_multiplot::CurveConfig::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_multiplot__CurveConfig.stringdata0))
        return static_cast<void*>(this);
    return Config::qt_metacast(_clname);
}

int rqt_multiplot::CurveConfig::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Config::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void rqt_multiplot::CurveConfig::titleChanged(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rqt_multiplot::CurveConfig::subscriberQueueSizeChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
