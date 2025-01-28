// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <span>
#include <map>
#include <units/base.h>
#include <units/math.h>
#include <wpi/array.h>

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

/*
 *  AUTOLOG macro can log the following data types:
 *      intrinsics, vectors of intrinsics, units, vectors of units, StructSerializable,
 *      std::optional<> wrapped types
 * 
 *  It uses the variable name as the field name and will add the units abbreviation for unit types.  
 */
#define AUTOLOG(key,v) DataLogger::Log( key + "/" + #v, v );

class DataLogger {
private:
    // This class is a singleton.
    static DataLogger *singleton;

    // Constructor is private
    DataLogger() {}
    static DataLogger& GetInstance();

public:

    // delete copy constructor
    DataLogger(const DataLogger& obj) = delete; 

    static void Log( const std::string& s, const std::string& val );

    template <class T>
     requires wpi::StructSerializable<T>
    static void Log( const std::string& s, const T& val );
    template <class T>
     requires wpi::StructSerializable<T>
    static void Log( const std::string& s, std::span<T> a );
    template <class T>
     requires wpi::StructSerializable<T>
    static void Log( const std::string& s, std::vector<T>& v );
    template <class T, size_t N>
     requires wpi::StructSerializable<T>
    static void Log( const std::string& s, wpi::array<T,N>& a );


        // A units library type.
    template <class UnitType>
    requires units::traits::is_unit_t<UnitType>::value
    static void Log( const std::string& s, const UnitType& val ) noexcept;

        // A vector of units library type.
    template <class UnitType>
    requires units::traits::is_unit_t<UnitType>::value
    static void Log( const std::string& s, const std::vector<UnitType>& vec ) noexcept;

        // A std::optional wrapped type.
    template<class T>
    static void Log( const std::string& s, const std::optional<T>& opt );

    static void Log( const std::string& s );

    static void LogMetadata( void );

private:
    void Send( const std::string& s, const double& val );
    void Send( const std::string& s, const int64_t& val );
    void Send( const std::string& s, const bool& val );
    void Send( const std::string& s, const std::string& val );

    template <class T>
     requires wpi::StructSerializable<T>
    void Send( const std::string& s, const T& val );
    template <class T>
     requires wpi::StructSerializable<T>
    void Send( const std::string& s, std::span<T> a );

    void SendNT( const std::string& s, const double& val );
    void SendNT( const std::string& s, const int64_t& val );
    void SendNT( const std::string& s, const bool& val );
    void SendNT( const std::string& s, const std::string& val );

    template <class T>
     requires wpi::StructSerializable<T>
    void SendNT( const std::string& s, const T& val );
    template <class T>
     requires wpi::StructSerializable<T>
    void SendNT( const std::string& s, std::span<T> a );

    wpi::log::DataLog *log;
    std::shared_ptr<nt::NetworkTable> nt_table;
    std::map<std::string, nt::Publisher*> nt_map;
    bool isFMSAttached;

    void SendMetadata( std::string_view s, std::string_view val );
};

template <class UnitType>
requires units::traits::is_unit_t<UnitType>::value
void DataLogger::Log( const std::string &s, const UnitType& val ) noexcept {
    DataLogger::Log( s + "(" + units::abbreviation(val) + ")", val.value() );
}

template <class UnitType>
requires units::traits::is_unit_t<UnitType>::value
void DataLogger::Log( const std::string &s, const std::vector<UnitType>& vec ) noexcept 
{
    static std::vector<double> a;
    
    a.clear();
    for( size_t i=0; i<vec.size(); ++i ) {
        a.push_back(vec[i].value());
    }
    DataLogger::Log( s + "(" + units::abbreviation(UnitType(0)) + ")", std::span<double>( a ) );
}

template<class T>
void DataLogger::Log( const std::string &s, const std::optional<T>& opt ) 
{
    if( opt.has_value() ) {
        DataLogger::Log( s, opt.value() );
    }
}

        // frc::Pose2d specialization
template<> void DataLogger::Log( const std::string &s, const std::optional<frc::Pose2d>& opt );


template <class T>
    requires wpi::StructSerializable<T>
void DataLogger::Log( const std::string& s, const T& val ) 
{
    if( GetInstance().isFMSAttached ) { 
        GetInstance().Send(s,val); 
    } else { 
        GetInstance().SendNT(s,val); 
    } 
}

template <class T>
 requires wpi::StructSerializable<T>
void DataLogger::Log( const std::string& s, std::span<T> a )
{
    if( GetInstance().isFMSAttached ) { 
        GetInstance().Send(s,a); 
    } else { 
        GetInstance().SendNT(s,a); 
    } 
}

template <class T>
 requires wpi::StructSerializable<T>
void DataLogger::Log( const std::string& s, std::vector<T>& v )
{
    Log( s, std::span<T>(v) );
}

template <class T, size_t N>
 requires wpi::StructSerializable<T>
void DataLogger::Log( const std::string& s, wpi::array<T,N>& a )
{
    Log( s, std::span<T>(a) );
}


template <class T>
    requires wpi::StructSerializable<T>
void DataLogger::Send( const std::string& s, const T& val )
{
    wpi::log::StructLogEntry<T> le{ *(log), s };
    le.Append( val );    
}

template <class T>
 requires wpi::StructSerializable<T>
void DataLogger::Send( const std::string& s, std::span<T> a )
{
    wpi::log::StructArrayLogEntry<T> le{ *(log), s };
    le.Append( a );    
}


template <class T>
requires wpi::StructSerializable<T>
void DataLogger::SendNT( const std::string& s, const T& val ) 
{
    nt::StructPublisher<T>* publisher;
    if( !nt_map.contains( s ) ) {
        publisher = new nt::StructPublisher<T>();
        *publisher = nt_table->GetStructTopic<T>( s ).Publish();
        nt_map[s] = publisher;
    } else {
        nt::Publisher *base = nt_map[ s ];
        publisher = (nt::StructPublisher<T>*) base;
    }
    publisher->Set( val );
}

template <class T>
 requires wpi::StructSerializable<T>
void DataLogger::SendNT( const std::string& s, std::span<T> a ) 
{
    nt::StructArrayPublisher<T>* publisher;
    if( !nt_map.contains( s ) ) {
        publisher = new nt::StructArrayPublisher<T>();
        *publisher = nt_table->GetStructArrayTopic<T>( s ).Publish();
        nt_map[s] = publisher;
    } else {
        nt::Publisher *base = nt_map[ s ];
        publisher = (nt::StructArrayPublisher<T>*) base;
    }
    publisher->Set( a );
}
