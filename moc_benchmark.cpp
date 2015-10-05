/****************************************************************************
** Meta object code from reading C++ file 'benchmark.h'
**
** Created: Sun Oct 4 21:59:06 2015
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "src/benchmark.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'benchmark.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_AspectRatioPixmapLabel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      24,   23,   23,   23, 0x0a,
      43,   23,   23,   23, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_AspectRatioPixmapLabel[] = {
    "AspectRatioPixmapLabel\0\0setPixmap(QPixmap)\0"
    "resizeEvent(QResizeEvent*)\0"
};

void AspectRatioPixmapLabel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        AspectRatioPixmapLabel *_t = static_cast<AspectRatioPixmapLabel *>(_o);
        switch (_id) {
        case 0: _t->setPixmap((*reinterpret_cast< const QPixmap(*)>(_a[1]))); break;
        case 1: _t->resizeEvent((*reinterpret_cast< QResizeEvent*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData AspectRatioPixmapLabel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject AspectRatioPixmapLabel::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_AspectRatioPixmapLabel,
      qt_meta_data_AspectRatioPixmapLabel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &AspectRatioPixmapLabel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *AspectRatioPixmapLabel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *AspectRatioPixmapLabel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_AspectRatioPixmapLabel))
        return static_cast<void*>(const_cast< AspectRatioPixmapLabel*>(this));
    return QLabel::qt_metacast(_clname);
}

int AspectRatioPixmapLabel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
static const uint qt_meta_data_Displayer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Displayer[] = {
    "Displayer\0\0update()\0"
};

void Displayer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Displayer *_t = static_cast<Displayer *>(_o);
        switch (_id) {
        case 0: _t->update(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData Displayer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Displayer::staticMetaObject = {
    { &AspectRatioPixmapLabel::staticMetaObject, qt_meta_stringdata_Displayer,
      qt_meta_data_Displayer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Displayer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Displayer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Displayer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Displayer))
        return static_cast<void*>(const_cast< Displayer*>(this));
    return AspectRatioPixmapLabel::qt_metacast(_clname);
}

int Displayer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = AspectRatioPixmapLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
static const uint qt_meta_data_Waiter[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
       8,    7,    7,    7, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_Waiter[] = {
    "Waiter\0\0updated()\0"
};

void Waiter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Waiter *_t = static_cast<Waiter *>(_o);
        switch (_id) {
        case 0: _t->updated(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData Waiter::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Waiter::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Waiter,
      qt_meta_data_Waiter, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Waiter::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Waiter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Waiter::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Waiter))
        return static_cast<void*>(const_cast< Waiter*>(this));
    return QThread::qt_metacast(_clname);
}

int Waiter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void Waiter::updated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
