#include "includes.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define EVAL_I2Cx_TIMEOUT_MAX                   3000

/* ˽�б��� ------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
uint32_t I2cxTimeout = EVAL_I2Cx_TIMEOUT_MAX;


/**
  * ��������: I2Cͨ�Ŵ�������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: һ����I2Cͨ�ų�ʱʱ���øú���
  */
static void BSP_I2C_MPU6050_Error (void)
{
  /* ����ʼ��I2Cͨ������ */
  HAL_I2C_DeInit(&hi2c2);
  
  /* ���³�ʼ��I2Cͨ������*/
  MX_I2C2_Init();
  printf("MPU6050 I2Cͨ�ų�ʱ������ ��������I2C...\n");
}

/**
  * ��������: ͨ��I2Cд��һ��ֵ��ָ���Ĵ�����
  * �������: Addr��I2C�豸��ַ
  *           Reg��Ŀ��Ĵ���
  *           Value��ֵ
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void BSP_I2C_MPU6050_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout);
  
  /* ���I2Cͨ��״̬ */
  if(status != HAL_OK)
  {
    /* ����I2Cͨ�Ŵ������� */
    BSP_I2C_MPU6050_Error();
  }
}

/**
  * ��������: ͨ��I2Cд��һ�����ݵ�ָ���Ĵ�����
  * �������: Addr��I2C�豸��ַ
  *           Reg��Ŀ��Ĵ���
  *           RegSize���Ĵ����ߴ�(8λ����16λ)
  *           pBuffer��������ָ��
  *           Length������������
  * �� �� ֵ: HAL_StatusTypeDef���������
  * ˵    ��: ��ѭ�����������һ����ʱʱ��
  */
HAL_StatusTypeDef BSP_I2C_MPU6050_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2cxTimeout); 

  /* ���I2Cͨ��״̬ */
  if(status != HAL_OK)
  {
    /* ����I2Cͨ�Ŵ������� */
    BSP_I2C_MPU6050_Error();
  }        
  return status;
}

/**
  * ��������: ͨ��I2C��ȡһ��ָ���Ĵ�������
  * �������: Addr��I2C�豸��ַ
  *           Reg��Ŀ��Ĵ���
  * �� �� ֵ: uint8_t���Ĵ�������
  * ˵    ��: ��
  */
uint8_t BSP_I2C_MPU6050_ReadData(uint16_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  
  status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);
 
  /* ���I2Cͨ��״̬ */
  if(status != HAL_OK)
  {
    /* ����I2Cͨ�Ŵ������� */
    BSP_I2C_MPU6050_Error();
  
  }
  return value;
}

/**
  * ��������: ͨ��I2C��ȡһ�μĴ������ݴ�ŵ�ָ���Ļ�������
  * �������: Addr��I2C�豸��ַ
  *           Reg��Ŀ��Ĵ���
  *           RegSize���Ĵ����ߴ�(8λ����16λ)
  *           pBuffer��������ָ��
  *           Length������������
  * �� �� ֵ: HAL_StatusTypeDef���������
  * ˵    ��: ��
  */
HAL_StatusTypeDef BSP_I2C_MPU6050_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hi2c2, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2cxTimeout);
  
  /* ���I2Cͨ��״̬ */
  if(status != HAL_OK)
  {
    /* ����I2Cͨ�Ŵ������� */
    BSP_I2C_MPU6050_Error();
  }        
  return status;
}

/**
  * ��������: ���I2C�豸�Ƿ���׼���ÿ���ͨ��״̬
  * �������: DevAddress��I2C�豸��ַ
  *           Trials�����Բ��Դ���
  * �� �� ֵ: HAL_StatusTypeDef���������
  * ˵    ��: ��
  */
HAL_StatusTypeDef BSP_I2C_MPU6050_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(&hi2c2, DevAddress, Trials, I2cxTimeout));
}
