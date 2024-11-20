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
#include <networktables/GenericEntry.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>

/*
 *  AUTOLOG macro can log the following:
 *      intrinsic data types, 
 *      vectors of intrinsic data types, 
 *      unit types,
 *      vectors of unit types,
 *      std::optional wrapped types
 * 
 *  It uses the variable name as the field name and will add the units abbreviation for unit types.  
 *  It can also log Pose2d() and the array of SwerveModuleState objects returned from the 
 *  kinematics.ToSwerveModuleStates() function.
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

        // Base Log() functions that other Log() functions call.
    static void Log( const std::string& s, double val );
    static void Log( const std::string& s, std::span<const double> a );
    static void Log( const std::string& s, int64_t val );
    static void Log( const std::string& s, std::span<const int64_t> a );
    static void Log( const std::string& s, bool val );
    static void Log( const std::string& s, std::span<const bool> a );
    static void Log( const std::string& s, const std::string& val );

        // Derived type Log() functions
        // These call the Log() function above taking a std::span<const double>
    static void Log( const std::string& s, const frc::Pose2d& p );
    static void Log( const std::string& s, const std::vector<frc::Pose2d>& pv );
    static void Log( const std::string& s, const wpi::array<frc::SwerveModuleState, 4U> &sms );

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
    void Send( const std::string& s, double val );
    void Send( const std::string& s, std::span<const double> a );
    void Send( const std::string& s, int64_t val );
    void Send( const std::string& s, std::span<const int64_t> a );
    void Send( const std::string& s, bool val );
    void Send( const std::string& s, std::span<const bool> a );
    void Send( const std::string& s, const std::string& val );

    void SendNT( const std::string& s, double val );
    void SendNT( const std::string& s, std::span<const double> a );
    void SendNT( const std::string& s, int64_t val );
    void SendNT( const std::string& s, std::span<const int64_t> a );
    void SendNT( const std::string& s, bool val );
    void SendNT( const std::string& s, std::span<const bool> a );
    void SendNT( const std::string& s, const std::string& val );

    wpi::log::DataLog *log;
    std::shared_ptr<nt::NetworkTable> nt_table;
    std::map<std::string, nt::GenericPublisher> nt_map;
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
void DataLogger::Log( const std::string &s, const std::vector<UnitType>& vec ) noexcept {
    static std::vector<double> a{256};
    
    a.clear();
    for( size_t i=0; i<vec.size(); ++i ) {
        a.push_back(vec[i].value());
    }
    DataLogger::Log( s + "(" + units::abbreviation(UnitType(0)) + ")", std::span<const double>( a ) );
}

template<class T>
void DataLogger::Log( const std::string &s, const std::optional<T>& opt ) {
    if( opt.has_value() ) {
        DataLogger::Log( s, opt.value() );
    }
}

        // frc::Pose2d specialization
template<> void DataLogger::Log( const std::string &s, const std::optional<frc::Pose2d>& opt );
