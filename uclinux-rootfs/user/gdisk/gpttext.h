/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef __GPTDATATEXT_H
#define __GPTDATATEXT_H

#include "gpt.h"

using namespace std;

class GPTDataTextUI : public GPTData {
   protected:
   public:
      GPTDataTextUI(void);
      GPTDataTextUI(string filename);
      ~GPTDataTextUI(void);

      // Extended (interactive) versions of some base-class functions
      WhichToUse UseWhichPartitions(void);
      int XFormDisklabel(void);

      // Request information from the user (& possibly do something with it)
      uint32_t GetPartNum(void);
      void ResizePartitionTable(void);
      void CreatePartition(void);
      void DeletePartition(void);
      void ChangePartType(void);
      void SetAttributes(uint32_t partNum);
      int SwapPartitions(void);
      int DestroyGPTwPrompt(void); // Returns 1 if user proceeds
      void ShowDetails(void);
      void MakeHybrid(void);
      int AssignPrimaryOrLogical(PartNotes& notes);
      int XFormToMBR(void); // convert GPT to MBR, wiping GPT afterwards. Returns 1 if successful
}; // class GPTDataTextUI

int GetMBRTypeCode(int defType);

#endif // __GPTDATATEXT_H
