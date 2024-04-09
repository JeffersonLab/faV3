/*********************************************
 *
 *  FADC Internal Trigger FADC Configuration and Control
 *  Routines.
 */

#ifndef EIEIO
#define EIEIO
#endif

uint32_t
faItrigStatus(int id, int sFlag)
{
  uint32_t status, config, twidth, wMask, wWidth, cMask, sum_th;
  uint32_t itrigCnt, trigOut;
  int vers, disabled, mode;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("faItrigStatus: ERROR : FADC in slot %d is not initialized \n",
	     id);
      return (ERROR);
    }

  /* Express Time in ns - 4ns/clk  */
  FAV3LOCK;
  status = vmeRead32(&FAV3p[id]->hitsum_status) & 0xffff;
  config = vmeRead32(&FAV3p[id]->hitsum_cfg) & 0xffff;
  twidth =
    (vmeRead32(&FAV3p[id]->hitsum_trig_width) & 0xffff) * FAV3_ADC_NS_PER_CLK;
  wMask = vmeRead32(&FAV3p[id]->hitsum_window_bits) & 0xffff;
  wWidth =
    (vmeRead32(&FAV3p[id]->hitsum_window_width) & 0xffff) * FAV3_ADC_NS_PER_CLK;
  cMask = vmeRead32(&FAV3p[id]->hitsum_coin_bits) & 0xffff;
  sum_th = vmeRead32(&FAV3p[id]->hitsum_sum_thresh) & 0xffff;
  itrigCnt = vmeRead32(&FAV3p[id]->internal_trig_scal);
  trigOut = vmeRead32(&FAV3p[id]->ctrl1) & FAV3_ITRIG_OUT_MASK;
  FAV3UNLOCK;

  vers = status & FAV3_ITRIG_VERSION_MASK;
  mode = config & FAV3_ITRIG_MODE_MASK;
  if(mode == FAV3_ITRIG_SUM_MODE)	/* If Sum mode then Live trigger always enabled */
    disabled = 0;
  else
    disabled = config & FAV3_ITRIG_ENABLE_MASK;


  printf("\n FADC Internal Trigger (HITSUM) Configuration: \n");
  printf("  (Mode: 0-Table 1-Coin 2-Window 4-Sum)\n");
  if(disabled)
    printf("     Hitsum Status      = 0x%04x    Config = 0x%04x   (Mode = %d - Disabled)\n",
	   status, config, mode);
  else
    printf("     Hitsum Status      = 0x%04x    Config = 0x%04x   (Mode = %d - Enabled)\n",
	   status, config, mode);

  printf("     Window  Input Mask = 0x%04x    Width = %5d ns\n", wMask,
	 wWidth);
  printf("     Coin    Input Mask = 0x%04x \n", cMask);
  printf("     Sum Mode Threshold = %d\n", sum_th);
  if(trigOut == FAV3_ITRIG_OUT_FP)
    printf("     Trigger Out  Width =  %5d ns (Front panel output)\n",
	   twidth);
  else
    printf("     Trigger Out  Width =  %5d ns (Output disabled)\n", twidth);
  printf("     Internal Triggers (Live) = %d\n", itrigCnt);

  return (config);
}

/************************************************************
 *
 *  Setup Internal Triggering
 *
 *   Four Modes of Operation (tmode)
 *     0) Table Mode
 *     1) Coincidence Mode
 *     2) Window Mode
 *     3) INVALID
 *     4) Sum Mode
 *
 *   wMask     = Mask of 16 channels to be enabled for Window Mode
 *   wWidth    = Width of trigger window before latching (in clocks)
 *   cMask     = Mask of 16 channels to be enabled for Coincidence Mode
 *   sumThresh = 10-12 bit threshold for Sum trigger to be latched
 *   tTable    = pointer to trigger table (65536 values) to be loaded
 */
int
faItrigSetMode(int id, int tmode, uint32_t wWidth, uint32_t wMask,
	       uint32_t cMask, uint32_t sumThresh, uint32_t * tTable)
{
  int ii;
  uint32_t config, stat, wTime;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("faItrigSetMode: ERROR : FADC in slot %d is not initialized \n",
	     id);
      return (ERROR);
    }

  /* Make sure we are not enabled or running */
  FAV3LOCK;
  config = vmeRead32(&FAV3p[id]->hitsum_cfg) & FAV3_ITRIG_CONFIG_MASK;
  FAV3UNLOCK;
  if((config & FAV3_ITRIG_ENABLE_MASK) == 0)
    {
      printf("faItrigSetMode: ERROR: Internal triggers are enabled - Disable first\n");
      return (ERROR);
    }

  if((tmode == FAV3_ITRIG_UNDEF_MODE) || (tmode > FAV3_ITRIG_SUM_MODE))
    {
      printf("faItrigSetMode: ERROR: Trigger mode (%d) out of range (tmode = 0-4)\n",
	     tmode);
      return (ERROR);
    }

  /* Check if we need to load a trigger table */
  if(tTable != NULL)
    {
      printf("faItrigSetMode: Loading trigger table from address 0x%lx \n",
	     (unsigned long) tTable);
      FAV3LOCK;
      vmeWrite32(&FAV3p[id]->s_adr, FAV3_SADR_AUTO_INCREMENT);
      vmeWrite32(&FAV3p[id]->hitsum_pattern, 0);	/* Make sure address 0 is not a valid trigger */
      for(ii = 1; ii <= 0xffff; ii++)
	{
	  if(tTable[ii])
	    vmeWrite32(&FAV3p[id]->hitsum_pattern, 1);
	  else
	    vmeWrite32(&FAV3p[id]->hitsum_pattern, 0);
	}
      FAV3UNLOCK;
    }

  switch (tmode)
    {
    case FAV3_ITRIG_SUM_MODE:
      /* Load Sum Threshhold if in range */
      FAV3LOCK;
      if((sumThresh > 0) && (sumThresh <= 0xffff))
	{
	  vmeWrite32(&FAV3p[id]->hitsum_sum_thresh, sumThresh);
	}
      else
	{
	  printf("faItrigSetMode: ERROR: Sum Threshold out of range (0<st<=0xffff)\n");
	  FAV3UNLOCK;
	  return (ERROR);
	}
      stat = (config & ~FAV3_ITRIG_MODE_MASK) | FAV3_ITRIG_SUM_MODE;
      vmeWrite32(&FAV3p[id]->hitsum_cfg, stat);
      FAV3UNLOCK;
      printf("faItrigSetMode: Configure for SUM Mode (Threshold = 0x%x)\n",
	     sumThresh);
      break;

    case FAV3_ITRIG_COIN_MODE:
      /* Set Coincidence Input Channels */
      FAV3LOCK;
      if((cMask > 0) && (cMask <= 0xffff))
	{
	  vmeWrite32(&FAV3p[id]->hitsum_coin_bits, cMask);
	}
      else
	{
	  printf("faItrigSetMode: ERROR: Coincidence channel mask out of range (0<cc<=0xffff)\n");
	  FAV3UNLOCK;
	  return (ERROR);
	}
      stat = (config & ~FAV3_ITRIG_MODE_MASK) | FAV3_ITRIG_COIN_MODE;
      vmeWrite32(&FAV3p[id]->hitsum_cfg, stat);
      FAV3UNLOCK;
      printf("faItrigSetMode: Configure for COINCIDENCE Mode (channel mask = 0x%x)\n",
	 cMask);
      break;

    case FAV3_ITRIG_WINDOW_MODE:
      /* Set Trigger Window width and channel mask */
      FAV3LOCK;
      if((wMask > 0) && (wMask <= 0xffff))
	{
	  vmeWrite32(&FAV3p[id]->hitsum_window_bits, wMask);
	}
      else
	{
	  printf("faItrigSetMode: ERROR: Trigger Window channel mask out of range (0<wc<=0xffff)\n");
	  FAV3UNLOCK;
	  return (ERROR);
	}
      if((wWidth > 0) && (wWidth <= FAV3_ITRIG_MAX_WIDTH))
	{
	  vmeWrite32(&FAV3p[id]->hitsum_window_width, wWidth);
	  wTime = 4 * wWidth;
	}
      else
	{
	  printf("faItrigSetMode: ERROR: Trigger Window width out of range (0<ww<=0x200)\n");
	  FAV3UNLOCK;
	  return (ERROR);
	}
      stat = (config & ~FAV3_ITRIG_MODE_MASK) | FAV3_ITRIG_WINDOW_MODE;
      vmeWrite32(&FAV3p[id]->hitsum_cfg, stat);
      FAV3UNLOCK;
      printf("faItrigSetMode: Configure for Trigger WINDOW Mode (channel mask = 0x%x, width = %d ns)\n",
	     wMask, wTime);
      break;

    case FAV3_ITRIG_TABLE_MODE:
      FAV3LOCK;
      stat = (config & ~FAV3_ITRIG_MODE_MASK) | FAV3_ITRIG_TABLE_MODE;
      vmeWrite32(&FAV3p[id]->hitsum_cfg, stat);
      FAV3UNLOCK;
      printf("faItrigSetMode: Configure for Trigger TABLE Mode\n");
    }

  return (OK);
}

/************************************************************
 *
 *  Setup Internal Trigger Table
 *    16 input channels can be latched to create a 16 bit
 *  lookup address (0x0001 - 0xffff) in memory. The value 0 or 1
 *  at that memory address determines if a trigger pulse will be
 *  generated (this is for Window or Table mode only)
 *
 *   table = pointer to an array of 65536 values (1 or 0) that
 *           will define a valid trigger or not.
 *      (if = NULL, then the default table is loaded - all
 *       input combinations will generate a trigger)
 */
int
faItrigInitTable(int id, uint32_t * table)
{
  int ii;
  uint32_t config;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("faItrigInitTable: ERROR : FADC in slot %d is not initialized \n",
	     id);
      return (ERROR);
    }

  /* Check and make sure we are not running */
  FAV3LOCK;
  config = vmeRead32(&FAV3p[id]->hitsum_cfg);
  if((config & FAV3_ITRIG_ENABLE_MASK) != FAV3_ITRIG_DISABLED)
    {
      printf("faItrigInitTable: ERROR: Cannot update Trigger Table while trigger is Enabled\n");
      FAV3UNLOCK;
      return (ERROR);
    }


  if(table == NULL)
    {
      /* Use default Initialization - all combinations of inputs will be a valid trigger */
      vmeWrite32(&FAV3p[id]->s_adr, FAV3_SADR_AUTO_INCREMENT);
      vmeWrite32(&FAV3p[id]->hitsum_pattern, 0);	/* Make sure address 0 is not a valid trigger */
      for(ii = 1; ii <= 0xffff; ii++)
	{
	  vmeWrite32(&FAV3p[id]->hitsum_pattern, 1);
	}

    }
  else
    {				/* Load specified table into hitsum FPGA */

      vmeWrite32(&FAV3p[id]->s_adr, FAV3_SADR_AUTO_INCREMENT);
      vmeWrite32(&FAV3p[id]->hitsum_pattern, 0);	/* Make sure address 0 is not a valid trigger */
      for(ii = 1; ii <= 0xffff; ii++)
	{
	  if(table[ii])
	    vmeWrite32(&FAV3p[id]->hitsum_pattern, 1);
	  else
	    vmeWrite32(&FAV3p[id]->hitsum_pattern, 0);
	}

    }
  FAV3UNLOCK;

  return (OK);
}




int
faItrigSetHBwidth(int id, uint16_t hbWidth, uint16_t hbMask)
{
  int ii;
  uint32_t config, hbval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("faItrigSetHBwidth: ERROR : FADC in slot %d is not initialized \n",
	     id);
      return (ERROR);
    }

  /* Check and make sure we are not running */
  FAV3UNLOCK;
  config = vmeRead32(&FAV3p[id]->hitsum_cfg);
  if((config & FAV3_ITRIG_ENABLE_MASK) != FAV3_ITRIG_DISABLED)
    {
      printf("faItrigSetHBwidth: ERROR: Cannot set HB widths while trigger is Enabled\n");
      FAV3UNLOCK;
      return (ERROR);
    }

  if(hbWidth > FAV3_ITRIG_MAX_HB_WIDTH)
    hbWidth = FAV3_ITRIG_MAX_HB_WIDTH;
  if(hbMask == 0)
    hbMask = 0xffff;		/* Set all Channels */

  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if((1 << ii) & hbMask)
	{
	  vmeWrite32(&FAV3p[id]->s_adr, ii);	/* Set Channel to Read/Write */
	  hbval =
	    vmeRead32(&FAV3p[id]->hitsum_hit_info) & ~FAV3_ITRIG_HB_WIDTH_MASK;
	  hbval = hbval | hbWidth;
	  vmeWrite32(&FAV3p[id]->hitsum_hit_info, hbval);	/* Set Value */
	}
    }
  FAV3UNLOCK;

  return (OK);
}

uint32_t
faItrigGetHBwidth(int id, uint32_t chan)
{
  uint32_t rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faItrigGetHBwidth: ERROR : FADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (0xffffffff);
    }

  if(chan >= FAV3_MAX_ADC_CHANNELS)
    {
      logMsg("faItrigGetHBwidth: ERROR : Channel # out of range (0-15)\n", 0,
	     0, 0, 0, 0, 0);
      return (0xffffffff);
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->s_adr, chan);	/* Set Channel */
  EIEIO;
  rval = vmeRead32(&FAV3p[id]->hitsum_hit_info) & FAV3_ITRIG_HB_WIDTH_MASK;	/* Get Value */
  FAV3UNLOCK;

  return (rval);
}

int
faItrigSetHBdelay(int id, uint16_t hbDelay, uint16_t hbMask)
{
  int ii;
  uint32_t config, hbval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("faItrigSetHBdelay: ERROR : FADC in slot %d is not initialized \n",
	     id);
      return (ERROR);
    }

  /* Check and make sure we are not running */
  FAV3LOCK;
  config = vmeRead32(&FAV3p[id]->hitsum_cfg);
  if((config & FAV3_ITRIG_ENABLE_MASK) != FAV3_ITRIG_DISABLED)
    {
      printf("faItrigSetHBdelay: ERROR: Cannot set HB delays while trigger is Enabled\n");
      FAV3UNLOCK;
      return (ERROR);
    }


  if(hbDelay > FAV3_ITRIG_MAX_HB_DELAY)
    hbDelay = FAV3_ITRIG_MAX_HB_DELAY;
  if(hbMask == 0)
    hbMask = 0xffff;		/* Set all Channels */

  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if((1 << ii) & hbMask)
	{
	  vmeWrite32(&FAV3p[id]->s_adr, ii);	/* Set Channel */
	  hbval =
	    vmeRead32(&FAV3p[id]->hitsum_hit_info) & ~FAV3_ITRIG_HB_DELAY_MASK;
	  hbval |= (hbDelay << 8);
	  vmeWrite32(&FAV3p[id]->hitsum_hit_info, hbval);	/* Set Value */
	}
    }
  FAV3UNLOCK;

  return (OK);
}

uint32_t
faItrigGetHBdelay(int id, uint32_t chan)
{
  uint32_t rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faItrigGetHBdelay: ERROR : FADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (0xffffffff);
    }

  if(chan > 15)
    {
      logMsg("faItrigGetHBdelay: ERROR : Channel # out of range (0-15)\n", 0,
	     0, 0, 0, 0, 0);
      return (0xffffffff);
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->s_adr, chan);	/* Set Channel */
  EIEIO;
  rval = (vmeRead32(&FAV3p[id]->hitsum_hit_info) & FAV3_ITRIG_HB_DELAY_MASK) >> 8;	/* Get Value */
  FAV3UNLOCK;

  return (rval);
}


void
faItrigPrintHBinfo(int id)
{
  int ii;
  uint32_t hbval[16], wval, dval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("faItrigPrintHBinfo: ERROR : FADC in slot %d is not initialized \n",
	     id);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->s_adr, ii);
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      vmeWrite32(&FAV3p[id]->s_adr, ii);
      hbval[ii] = vmeRead32(&FAV3p[id]->hitsum_hit_info) & FAV3_ITRIG_HB_INFO_MASK;	/* Get Values */
    }
  FAV3UNLOCK;

  printf(" HitBit (width,delay) in nsec for FADC Inputs in slot %d:", id);
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      wval = ((hbval[ii] & FAV3_ITRIG_HB_WIDTH_MASK) + 1) * FAV3_ADC_NS_PER_CLK;
      dval =
	(((hbval[ii] & FAV3_ITRIG_HB_DELAY_MASK) >> 8) + 7) * FAV3_ADC_NS_PER_CLK;
      if((ii % 4) == 0)
	printf("\n");
      printf("Chan %2d: %4d,%3d  ", (ii + 1), wval, dval);
    }
  printf("\n");

}

uint32_t
faItrigSetOutWidth(int id, uint16_t itrigWidth)
{
  uint32_t retval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faItrigSetOutWidth: ERROR : FADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (0xffffffff);
    }

  if(itrigWidth > FAV3_ITRIG_MAX_WIDTH)
    itrigWidth = FAV3_ITRIG_MAX_WIDTH;

  FAV3LOCK;
  if(itrigWidth)
    vmeWrite32(&FAV3p[id]->hitsum_trig_width, itrigWidth);

  EIEIO;
  retval = vmeRead32(&FAV3p[id]->hitsum_trig_width) & 0xffff;
  FAV3UNLOCK;

  return (retval);
}

void
faItrigEnable(int id, int eflag)
{
  uint32_t rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faItrigEnable: ERROR : FADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->hitsum_cfg);
  rval &= ~(FAV3_ITRIG_DISABLED);

  vmeWrite32(&FAV3p[id]->hitsum_cfg, rval);

  if(eflag)
    {				/* Enable Live trigger to Front Panel Output */
      vmeWrite32(&FAV3p[id]->ctrl1, vmeRead32(&FAV3p[id]->ctrl1)
		 | (FAV3_ENABLE_LIVE_TRIG_OUT | FAV3_ENABLE_TRIG_OUT_FP));
    }
  FAV3UNLOCK;

}

void
faItrigDisable(int id, int dflag)
{
  uint32_t rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faItrigDisable: ERROR : FADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->hitsum_cfg);
  rval |= FAV3_ITRIG_DISABLED;

  vmeWrite32(&FAV3p[id]->hitsum_cfg, rval);

  if(dflag)
    {				/* Disable Live trigger to Front Panel Output */
      rval = vmeRead32(&FAV3p[id]->ctrl1);
      rval &= ~(FAV3_ENABLE_LIVE_TRIG_OUT | FAV3_ENABLE_TRIG_OUT_FP);
      vmeWrite32(&FAV3p[id]->ctrl1, rval);
    }
  FAV3UNLOCK;

}


int
faItrigGetTableVal(int id, uint16_t pMask)
{
  int rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faItrigGetTableVal: ERROR : FADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->s_adr, pMask);
  EIEIO;			/* Make sure write comes before read */
  rval = vmeRead32(&FAV3p[id]->hitsum_pattern) & 0x1;
  FAV3UNLOCK;

  return (rval);
}

void
faItrigSetTableVal(int id, uint16_t tval, uint16_t pMask)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faItrigSetTableVal: ERROR : FADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->s_adr, pMask);
  if(tval)
    vmeWrite32(&FAV3p[id]->hitsum_pattern, 1);
  else
    vmeWrite32(&FAV3p[id]->hitsum_pattern, 0);
  FAV3UNLOCK;

}
