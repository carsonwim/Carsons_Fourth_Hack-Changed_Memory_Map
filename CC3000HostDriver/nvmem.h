//
//#ifndef __NVRAM_H__
//#define __NVRAM_H__
//
//#include "cc3000_common.h"
//
//
////*****************************************************************************
////
//// If building with a C++ compiler, make all of the definitions in this header
//// have a C binding.
////
////*****************************************************************************
//#ifdef  __cplusplus
//extern "C" {
//#endif
//
//
////*****************************************************************************
////
////! \addtogroup nvmem_api
////! @{
////
////*****************************************************************************
//
///****************************************************************************
//**
//**	Definitions for File IDs
//**
//****************************************************************************/
///* NVMEM file ID */
//#define NVMEM_NVS_FILEID 							(0)
//#define NVMEM_NVS_SHADOW_FILEID 					(1)
//#define NVMEM_WLAN_CONFIG_FILEID 					(2)
//#define NVMEM_WLAN_CONFIG_SHADOW_FILEID 			(3)
//#define NVMEM_WLAN_DRIVER_SP_FILEID					(4)
//#define NVMEM_WLAN_FW_SP_FILEID						(5)
//#define NVMEM_MAC_FILEID 							(6)
//#define NVMEM_FRONTEND_VARS_FILEID 					(7)
//#define NVMEM_IP_CONFIG_FILEID 						(8)
//#define NVMEM_IP_CONFIG_SHADOW_FILEID 				(9)
//#define NVMEM_BOOTLOADER_SP_FILEID 					(10)
//#define NVMEM_RM_FILEID			 					(11)
//
///*  max entry in order to invalid nvmem              */
//#define NVMEM_MAX_ENTRY                              (14)
//
//
/////*****************************************************************************
//// * \brief Read data from nvmem
//// *
//// * Reads data from the file referred by the ulFileId parameter.
//// * Reads data from file ulOffset till len. Err if the file can't
//// * be used, is invalid, or if the read is out of bounds.
//// *
//// *
//// * \param[in] ulFileId   nvmem file id:\n
//// * NVMEM_NVS_FILEID, NVMEM_NVS_SHADOW_FILEID,
//// * NVMEM_WLAN_CONFIG_FILEID, NVMEM_WLAN_CONFIG_SHADOW_FILEID,
//// * NVMEM_WLAN_DRIVER_SP_FILEID, NVMEM_WLAN_FW_SP_FILEID,
//// * NVMEM_MAC_FILEID, NVMEM_FRONTEND_VARS_FILEID,
//// * NVMEM_IP_CONFIG_FILEID, NVMEM_IP_CONFIG_SHADOW_FILEID,
//// * NVMEM_BOOTLOADER_SP_FILEID or NVMEM_RM_FILEID.
//// * \param[in] ulLength   number of bytes to read
//// * \param[in] ulOffset   ulOffset in file from where to read
//// * \param[out] buff    output buffer pointer
//// *
//// * \return	    number of bytes read.
//// *
//// * \sa
//// * \note
//// * \warning
//// *
//// *****************************************************************************/
////
////extern signed long nvmem_read(unsigned long file_id, unsigned long length, unsigned long offset, unsigned char *buff);
//
//
//
//
/////*****************************************************************************
//// * \brief Write MAC address.
//// *
//// * Write MAC address to EEPROM.
//// * mac address as appears over the air (OUI first)
//// *
//// * \param[in] mac  mac address:\n
//// *
//// * \return	  on succes 0, error otherwise.
//// *
//// * \sa
//// * \note
//// * \warning
//// *
//// *****************************************************************************/
////extern	unsigned char nvmem_set_mac_address(unsigned char *mac);
//
//
/////*****************************************************************************
//// * \brief Read MAC address.
//// *
//// * Read MAC address from EEPROM.
//// * mac address as appears over the air (OUI first)
//// *
//// * \param[out] mac  mac address:\n
//// *
//// * \return	  on succes 0, error otherwise.
//// *
//// * \sa
//// * \note
//// * \warning
//// *
//// *****************************************************************************/
////extern	unsigned char nvmem_get_mac_address(unsigned char *mac);
//
//
/////*****************************************************************************
//// * \brief Write data to nvmem.
//// *
//// * program a patch to a specific file ID.
//// * The SP data is assumed to be continuous in memory.
//// * Actual programming is applied in 150 bytes portions (SP_PORTION_SIZE).
//// *
//// * \param[in] ulFileId   nvmem file id:\n
//// * NVMEM_WLAN_DRIVER_SP_FILEID,
//// * NVMEM_WLAN_FW_SP_FILEID,
//// * \param[in] spLength    number of bytes to write
//// * \param[in] spData      SP data to write
//// *
//// * \return	  on succes 0, error otherwise.
//// *
//// * \sa
//// * \note
//// * \warning
//// *
//// *****************************************************************************/
////extern	unsigned char nvmem_write_patch(unsigned long ulFileId, unsigned long spLength, const unsigned char *spData);
//
//
/////*****************************************************************************
//// * \brief Read patch version.
//// *
//// * read package version (WiFi FW patch, driver-supplicant-NS patch, bootloader patch)
//// *
//// * \param[out] patchVer    first number indicates package ID and the second number indicates package build number
//// *
//// * \return	  on succes 0, error otherwise.
//// *
//// * \sa
//// * \note
//// * \warning
//// *
//// *****************************************************************************/
////#ifndef CC3000_TINY_DRIVER
////extern	unsigned char nvmem_read_sp_version(unsigned char* patchVer);
////#endif
//
////*****************************************************************************
////
//// Mark the end of the C bindings section for C++ compilers.
////
////*****************************************************************************
//
//
////*****************************************************************************
////
//// Close the Doxygen group.
////! @}
////
////*****************************************************************************
//
//
//#ifdef  __cplusplus
//}
//#endif // __cplusplus
//
//#endif // __NVRAM_H__
