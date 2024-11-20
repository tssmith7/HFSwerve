// Log data to the Data Log file on the RoboRIO
//

#include <fstream>

#include <wpi/DataLog.h>
#include <frc/Filesystem.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/BooleanArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/GenericEntry.h>

#include "DataLogger.h"

#define SEND_LOG_DATA(s,val) \
    if( GetInstance().isFMSAttached ) { \
        GetInstance().Send(s,val); \
    } else { \
        GetInstance().SendNT(s,val); \
    } 


DataLogger* DataLogger::singleton = nullptr;

DataLogger& DataLogger::GetInstance() {
    // If there is no instance of class
    // then we can create an instance.
    if (singleton == nullptr)  {
        singleton = new DataLogger();
        singleton->log = &frc::DataLogManager::GetLog();
        singleton->nt_table = nt::NetworkTableInstance::GetDefault().GetTable("");
        singleton->isFMSAttached = frc::DriverStation::IsFMSAttached();
    }
        
    return *singleton;
}

void DataLogger::Log( const std::string& s, double val ) { SEND_LOG_DATA(s,val) }
void DataLogger::Log( const std::string& s, std::span<const double> a ) { SEND_LOG_DATA(s,a) }
void DataLogger::Log( const std::string& s, int64_t val ) { SEND_LOG_DATA(s,val) }
void DataLogger::Log( const std::string& s, std::span<const int64_t> a ) { SEND_LOG_DATA(s,a) }
void DataLogger::Log( const std::string& s, bool val ) { SEND_LOG_DATA(s,val) }
void DataLogger::Log( const std::string& s, std::span<const bool> a ) { SEND_LOG_DATA(s,a) }
void DataLogger::Log( const std::string& s, const std::string& val ) { SEND_LOG_DATA(s,val) }

void DataLogger::Log( const std::string& s, const frc::Pose2d& p ) {
    static double a[3];

    a[0] = p.X().value();
    a[1] = p.Y().value();
    a[2] = p.Rotation().Radians().value();

    Log( s, std::span{a} );
}

void DataLogger::Log( const std::string& s, const std::vector<frc::Pose2d>& pv ) {
    static std::vector<double> a{256};
    a.clear();
    for( size_t i=0; i<pv.size(); ++i ) {
        a.push_back( pv[i].X().value() );
        a.push_back( pv[i].Y().value() );
        a.push_back( pv[i].Rotation().Radians().value() );
    }
    return Log( s, std::span{ a } );
}

void DataLogger::Log( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms ) {
    static double a[8];

    if( sms.empty() ) {
        Log( s, std::span<const double>{} );
    } 

    for( int i=0; i<4; ++i ) {
        a[2*i] = sms[i].angle.Radians().value(); 
        a[2*i + 1] = sms[i].speed.value();
    }

    Log( s, std::span{a} );
}

template<>
void DataLogger::Log( const std::string &s, const std::optional<frc::Pose2d>& opt ) {
    if( opt.has_value() ) {
        DataLogger::Log( s, opt.value() );
    } else {
        DataLogger::Log( s, std::vector<double>{} );
    }
}

/**
 * Pass thru to the frc::DataLogManager::Log() command.
*/
void DataLogger::Log( const std::string &s ) {
    frc::DataLogManager::Log( s );
}





void DataLogger::Send( const std::string& s, double val ) { 
    wpi::log::DoubleLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, std::span<const double> a ) { 
    wpi::log::DoubleArrayLogEntry le{ *(log), s };
    le.Append( a );
}

void DataLogger::Send( const std::string& s, int64_t val ) { 
    wpi::log::IntegerLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, std::span<const int64_t> a ) { 
    wpi::log::IntegerArrayLogEntry le{ *(log), s };
    le.Append( a );
}

void DataLogger::Send( const std::string& s, bool val ) {
    wpi::log::BooleanLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, std::span<const bool> a ) {
    wpi::log::BooleanArrayLogEntry le{ *(log), s };
    le.Append( a );
}

void DataLogger::Send( const std::string& s, const std::string& val ) { 
    wpi::log::StringLogEntry le{ *(log), s };
    le.Append( val );
}





void DataLogger::SendNT( const std::string& s, double val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetDoubleTopic( s ).GenericPublish( "double" );
    }
    nt_map[s].SetDouble( val );
}

void DataLogger::SendNT( const std::string& s, std::span<const double> a ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetDoubleArrayTopic( s ).GenericPublish( "double[]" );
    }
    nt_map[s].SetDoubleArray( a );
}

void DataLogger::SendNT( const std::string &s, int64_t val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetIntegerTopic( s ).GenericPublish( "int" );
    }
    nt_map[s].SetInteger( val );
}

void DataLogger::SendNT( const std::string& s, std::span<const int64_t> a ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetIntegerArrayTopic( s ).GenericPublish( "int[]" );
    }
    nt_map[s].SetIntegerArray( a );
}

void DataLogger::SendNT( const std::string &s, bool val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetBooleanTopic( s ).GenericPublish( "boolean" );
    }
    nt_map[s].SetBoolean( val );
}

void DataLogger::SendNT( const std::string& s, std::span<const bool> a ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetBooleanArrayTopic( s ).GenericPublish( "boolean[]" );
    }
    nt_map[s].SetBooleanArray( a );
}

void DataLogger::SendNT( const std::string &s, const std::string &val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetStringTopic( s ).GenericPublish( "string" );
    }
    nt_map[s].SetString( val );
}


void DataLogger::LogMetadata( void ) {
        // Open the buildinfo.txt file and write the Metadata to the log file
    std::ifstream binfo;
    std::string fname;
    char line[256];

    fname = frc::filesystem::GetDeployDirectory() + "/buildinfo.txt";

    binfo.open( fname, std::ios::in );
    if( binfo.is_open() ) {
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "BUILD_DATE", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_REPO", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_BRANCH", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_VERSION", line );
        binfo.close();
    } else {
        Log( "Cannot open Metadata file: " + fname );
    }
}

void DataLogger::SendMetadata( std::string_view s, std::string_view val ) {
        // AdvantageScope Chops off leading Character of the name so we add an underscore.
        // Not sure why
    std::string id = "RealMetadata/_";
    id += s;
    wpi::log::StringLogEntry le{ *(log), id };
    le.Append( val );
}
