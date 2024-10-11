// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2022 by Laurent Itti, the University of Southern
// California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
//
// This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
// redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
// Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details.  You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
//
// Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 - USA.
// Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*! \file */

// This is only available on JeVoisPro
#ifdef JEVOIS_PRO

#include <jevois/GPU/GUIeditor.H>
#include <jevois/GPU/GUIhelper.H>
#include <jevois/Core/Module.H>
#include <jevois/Core/Engine.H>
#include <imgui-filebrowser/imfilebrowser.h>
#include <fstream>

#include <jevois/Debug/Log.H>

// ##############################################################################################################
jevois::GUIeditor::GUIeditor(GUIhelper * helper, std::string const & imguiid,
                             std::vector<jevois::EditorItem> && fixeditems, std::string const & scanpath,
                             std::string const & prefix, std::set<std::string> && extensions) :
    TextEditor(), itsHelper(helper), itsId(imguiid), itsItems(fixeditems), itsNumFixedItems(fixeditems.size()),
    itsScanPath(scanpath), itsPrefix(prefix), itsExtensions(extensions),
    itsBrowser(new ImGui::FileBrowser(ImGuiFileBrowserFlags_EnterNewFilename | ImGuiFileBrowserFlags_CreateNewDir))
{
  TextEditor::SetSaveCallback([this]() { saveFile(); } ); // to enable Ctrl-S saving

  itsBrowser->SetTitle("Select a file to open or create...");
  itsBrowser->SetPwd(JEVOIS_SHARE_PATH);
  //itsBrowser->SetTypeFilters({ ".cfg", ".py", ".txt", ".C", ".H" });
}

// ##############################################################################################################
jevois::GUIeditor::~GUIeditor()
{ }

// ##############################################################################################################
void jevois::GUIeditor::refresh()
{
  // 如果我们打开了一个文件，但它不是我们的固定项目之一，我们希望保持该文件打开：
  bool keep_current = false; EditorItem current_item;
  if (itsCurrentItem >= int(itsNumFixedItems)) { current_item = itsItems[itsCurrentItem]; keep_current = true; }
  
  // 删除所有动态文件，保留固定文件：
  itsItems.resize(itsNumFixedItems);

  // 我们是否有当前模块的 CMakeLists？
  if (itsItems[0].filename == "*")
  {
    jevois::VideoMapping const & vm = itsHelper->engine()->getCurrentVideoMapping();
    if (std::filesystem::exists(vm.cmakepath()))
      itsItems.emplace_back(EditorItem {"#", "Module's CMakeLists.txt", EditorSaveAction::Compile } );
  }
  
  // Rescan recursively:
  for (auto const & dent : std::filesystem::recursive_directory_iterator(itsScanPath))
  {
    if (dent.is_regular_file())
    {
      std::filesystem::path const path = dent.path();
      
      // 检查扩展是否是我们想要的：
      if (itsExtensions.find(path.extension()) == itsExtensions.end()) continue;

      // Create an entry:
      itsItems.emplace_back(EditorItem { path, itsPrefix + path.string(), EditorSaveAction::Reload });
    }
  }

  // Keep the current item?
  if (keep_current) itsItems.emplace_back(std::move(current_item));
  
  // 为文件浏览器/创建添加一个条目：
  itsItems.emplace_back(EditorItem { "**", "Browse / Create file...", EditorSaveAction::Reload });
  
  // Update index of current file:
  bool not_found = true;

  for (int i = 0; auto const & item : itsItems)
    if (item.filename == itsFilename)
    {
      itsCurrentItem = i;
      itsNewItem = i;
      not_found = false;
      break;
    }
    else ++i;

  // 如果当前打开的文件不再在我们的列表中，则加载文件 0：
  if (not_found)
  {
    itsNewItem = 0;
    itsWantLoad = true;
    // 如果我们有一些编辑，我们将要求保存，然后可能重新加载模块（或重新启动等）。 防止要求重新加载：
    itsOverrideReloadModule = IsEdited();
  }
}

// ##############################################################################################################
void jevois::GUIeditor::draw()
{
  // 为 imgui 创建组合条目：
  char const * items[itsItems.size()];
  for (int i = 0; EditorItem const & c : itsItems) items[i++] = c.displayname.c_str();

  // 检查用户是否尝试选择其他文件：
  if (ImGui::Combo(("##"+itsId+"editorcombo").c_str(), &itsNewItem, items, itsItems.size())) itsWantLoad = true;
  
  // 想要加载新文件？先检查是否需要保存当前文件：
  if (itsWantLoad && itsWantAction == false)
  {
    if (IsEdited())
    {
      static int discard_edits_default = 0;
      int ret = itsHelper->modal("Discard edits?", "File was edited. Discard all edits? This cannot be undone.",
                                 &discard_edits_default, "Discard", "Save");
      switch (ret)
      {
      case 1: itsWantLoad = false; itsOkToLoad = true; break; // Discard selected
      case 2: saveFile(); /* itsWantAction = false; */ break; // save selected
      default: break;  // Need to wait
      }
    }
    else
    {
      itsWantLoad = false;
      itsOkToLoad = true;
    }
  }
  
  // 保存后需要执行操作吗？
  if (itsWantAction)
  {
    switch (itsItems[itsCurrentItem].action)
    {
      // --------------------------------------------------
    case jevois::EditorSaveAction::None:
      itsWantAction = false;
      break;
      
      // --------------------------------------------------
    case jevois::EditorSaveAction::Reload:
    {
      // 如果 refresh() 请求覆盖，则跳过，通常是因为我们加载了新模块，但是旧模块中的配置文件已打开，因此我们要求保存，
	  // 现在我们不想再次要求重新加载：
      if (itsOverrideReloadModule)
      {
        itsOverrideReloadModule = false;
        itsWantAction = false;
        itsOkToLoad = itsWantLoad;
        break;
      }

      // 询问是否立即或稍后重新加载模块：
      static int reload_default = 0;
      int ret = itsHelper->modal("Reload Module?", "Reload Machine Vision Module for changes to take effect?",
                                 &reload_default, "Reload", "Later");
      switch (ret)
      {
      case 1:  // Reload selected
        itsHelper->engine()->requestSetFormat(-1);
        itsWantAction = false;
        itsOkToLoad = itsWantLoad;
        break;
        
      case 2: // 选择稍后：我们不再需要操作 
        itsWantAction = false;
        itsOkToLoad = itsWantLoad;
        break;

      default: break; // need to wait
      }
    }
    break;
    
    // --------------------------------------------------
    case jevois::EditorSaveAction::Reboot:
    {
      int ret = itsHelper->modal("Restart?", "Restart JeVois-Pro for changes to take effect?",
                                 nullptr, "Restart", "Later");
      switch (ret)
      {
      case 1: // Reboot selected
        itsHelper->engine()->reboot();
        itsWantAction = false;
        break;
        
      case 2: // 选择稍后：我们不再需要操作 
        itsWantAction = false;
        break;

      default: break;  // Need to wait
      }
    }
    break;

    // --------------------------------------------------
    case jevois::EditorSaveAction::RefreshMappings:
    {
      itsHelper->engine()->reloadVideoMappings();
      itsWantAction = false;
    }
    break;

    // --------------------------------------------------
    case jevois::EditorSaveAction::Compile:
    {
      // 询问是否立即编译模块还是稍后编译：
      static int compile_default = 0;
      int ret = itsHelper->modal("Compile Module?", "Compile Machine Vision Module for changes to take effect?",
                                 &compile_default, "Compile", "Later");
      switch (ret)
      {
      case 1:  // Compile selected
        itsHelper->startCompilation();
        itsWantAction = false;
        itsOkToLoad = itsWantLoad;
        break;
        
      case 2: // Later selected: we don't want action anymore
        itsWantAction = false;
        itsOkToLoad = itsWantLoad;
        break;
        
      default: break; // need to wait
      }
    }
    break;
    }
  }
  
  // Ready to load a new file?
  if (itsOkToLoad)
  {
    // Do we want to browse or create a new file?
    if (itsItems[itsNewItem].filename == "**")
    {
      ImGui::PushStyleColor(ImGuiCol_PopupBg, 0xf0ffe0e0);
      
      if (itsBrowser->IsOpened() == false)
      {
        itsBrowser->Open();
        itsBrowser->Display();
      }
      else
      {
        itsBrowser->Display();

        if (itsBrowser->HasSelected())
        {
          std::filesystem::path const fn = itsBrowser->GetSelected();
          
          if (std::filesystem::exists(fn))
            loadFileInternal(fn, "Could not load " + fn.string()); // load with error and read-only on fail
          else
            loadFileInternal(fn, ""); // load with no error and read-write on fail (create new file)
          
          itsBrowser->Close();
          itsBrowser->Display();
        }
        
        // Clicking "Cancel" in the browser just closes the popup:
        if (itsBrowser->IsOpened() == false)
        {
          itsOkToLoad = false; // Record that we don't want to load anymore:
          itsNewItem = itsCurrentItem; // Snap back the combo selector to the current item
          itsBrowser->Close();
          itsBrowser->Display();
        }
      }
      ImGui::PopStyleColor();
    }
    else
    {
      // Load the file for itsNewItem:
      itsCurrentItem = itsNewItem;
      loadFileInternal(itsItems[itsCurrentItem].filename, "");
    }
  }
  
  // Add a pop-up menu for editor actions:
  bool const ro = IsReadOnly();
  ImGui::SameLine();
  if (ImGui::Button("...")) ImGui::OpenPopup("editor_actions");
  if (ImGui::BeginPopup("editor_actions"))
  {
    constexpr int ok = ImGuiSelectableFlags_None;
    constexpr int disa = ImGuiSelectableFlags_Disabled;

    if (ImGui::Selectable("Save   [Ctrl-S]", false, !ro && IsEdited() ? ok : disa)) saveFile();

    ImGui::Separator();

    if (ImGui::Selectable("Undo   [Ctrl-Z]", false, !ro && CanUndo() ? ok : disa)) Undo();
    if (ImGui::Selectable("Redo   [Ctrl-Y]", false, !ro && CanRedo() ? ok : disa)) Redo();
      
    ImGui::Separator();
      
    if (ImGui::Selectable("Copy   [Ctrl-C]", false, HasSelection() ? ok : disa)) Copy();
    if (ImGui::Selectable("Cut    [Ctrl-X]", false, !ro && HasSelection() ? ok : disa)) Cut();
    if (ImGui::Selectable("Delete [Del]", false, !ro && HasSelection() ? ok : disa)) Delete();
    if (ImGui::Selectable("Paste  [Ctrl-V]", false, !ro && ImGui::GetClipboardText()!=nullptr ? ok : disa)) Paste();
      
    ImGui::Separator();
      
    ImGui::Selectable("More shortcuts...", false, disa);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("[Ctrl-A]          Select all\n"
                        "[PgUp/PgDn]       Move one page up/down\n"
                        "[Home]            Move to start of line\n"
                        "[End]             Move to end of line\n"
                        "[Ctrl-Home]       Move to start of file\n"
                        "[Ctrl-End]        Move to end of file\n"
                        "[Ctrl-Left/Right] Move left/right one word\n"
                        "[Ins]             Toggle overwrite mode\n"
                        "[Alt-Bksp]        Undo (same as [Ctrl-Z])\n"
                        "[Ctrl-Ins]        Copy (same as [Ctrl-C])\n"
                        "[Shift-Ins]       Paste (same as [Ctrl-V])\n"
                        "[Shift-Del]       Cut (same as [Ctrl-X])\n"
                        "[Shift-Cursor]    Select while moving cursor (up, down, left, right, home, end)\n"
                        "[Mouse-Drag]      Select with mouse\n"
                        );
    
    ImGui::EndPopup();
  }

  // 如果我们可以读/写，则绘制一个保存按钮：
  if (ro == false)
  {
    ImGui::SameLine();
    ImGui::TextUnformatted("   "); ImGui::SameLine();
    if (ImGui::Button("Save")) saveFile();
  }
  
  ImGui::Separator();

  // 在子窗口中呈现编辑器，以便它可以正确滚动：
  auto cpos = GetCursorPosition();

  ImGui::Text("%6d/%-6d %6d lines  | %s | %s | %s", cpos.mLine + 1, cpos.mColumn + 1, GetTotalLines(),
              IsOverwrite() ? "Ovr" : "Ins",
              IsEdited() ? "*" : " ",
              GetLanguageDefinition().mName.c_str());
  
  Render("JeVois-Pro Editor");
}

// ##############################################################################################################
void jevois::GUIeditor::loadFile(std::filesystem::path const & fn)
{
  // 加载将在主循环中进行。这里我们只需创建一个新项目：
  for (int i = 0; EditorItem const & item : itsItems)
    if (item.filename == fn) { itsNewItem = i; itsWantLoad = true; return; } else ++i;

  // 尚未在我们的项目列表中，请创建一个新的。为我们的新文件添加一个条目。如果它是可编译的，则保存时的操作应该是编译，
  // 否则应该是重新加载模块：
  EditorSaveAction action = EditorSaveAction::Reload;
  if (fn.filename() == "CMakeLists.txt")
    action = EditorSaveAction::Compile;
  else if (fn.filename() == "jevoispro-fan.service")
    action = EditorSaveAction::Reboot;
  else
  {
    std::string const ext = fn.extension().string();
    if (ext == ".C" || ext == ".H" || ext == ".cpp" || ext == ".hpp" || ext == ".c" || ext == ".h")
      action = EditorSaveAction::Compile;
  }
  itsItems.emplace_back(EditorItem { fn, "File " + fn.string(), action });
  
  itsNewItem = itsItems.size() - 1;
  itsWantLoad = true;
}

// ##############################################################################################################
void jevois::GUIeditor::loadFileInternal(std::filesystem::path const & fpath, std::string const & failt)
{
  std::filesystem::path fn = fpath; std::string failtxt = failt; bool special_path = false;
  
  if (fpath == "*")
  {
    // 如果文件名为 "*"，则替换为模块的源代码名称：
    jevois::VideoMapping const & vm = itsHelper->engine()->getCurrentVideoMapping();
    fn = vm.srcpath();
    failtxt = "Could not open Module's source code";
    special_path = true;
  }
  else if (fpath == "#")
  {
    // 如果文件名是 "#"，则用模块的 CMakeLists.txt 替换：
    jevois::VideoMapping const & vm = itsHelper->engine()->getCurrentVideoMapping();
    fn = vm.cmakepath();
    failtxt = "Could not open Module's CMakeLists.txt";
    special_path = true;
  }
  else if (fpath.is_relative())
  {
    // 如果路径是相对的，则使其在模块的路径内（如果有）：
    auto m = itsHelper->engine()->module();
    if (m) fn = m->absolutePath(fpath);
    special_path = true;
  }

  if (fn != fpath) LINFO("Loading " << fn << " ... [" << fpath << ']'); else LINFO("Loading " << fn << " ...");

  bool got_it = false;
  EditorSaveAction action = EditorSaveAction::Reload;
    
  std::ifstream t(fn);
  if (t.good())
  {
    // 加载整个文件并将其设置为我们的文本：
    std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    SetText(str);

    // 这是我们下拉列表中的已知文件吗？否则我们需要创建一个新项目：
    for (int i = 0; EditorItem const & item : itsItems)
      if (item.filename == fpath) { itsCurrentItem = i; got_it = true; break; } else ++i;

    // 根据文件扩展名设置语言、只读和可能的操作。如果同一目录中存在 CMakeLists.txt（例如，新创建或克隆的模块，不包括 
	// jevoisbase 模块），则 C++/C 源文件是可编辑的，然后在保存时会触发编译操作：
    if (fn.filename() == "CMakeLists.txt")
    {
      SetLanguageDefinition(TextEditor::LanguageDefinition::CMake());
      SetReadOnly(false);
      action = EditorSaveAction::Compile;
    }
    else
    {
      std::filesystem::path const ext = fn.extension();
      std::filesystem::path cmak = fn; cmak.remove_filename(); cmak /= "CMakeLists.txt";
      bool has_cmake = std::filesystem::exists(cmak);
      
      if (ext == ".py")
      {
        SetLanguageDefinition(TextEditor::LanguageDefinition::Python());
        SetReadOnly(false);
      }
      else if (ext == ".C" || ext == ".H" || ext == ".cpp" || ext == ".hpp")
      {
        SetLanguageDefinition(TextEditor::LanguageDefinition::CPlusPlus());
        SetReadOnly(! has_cmake);
        action = EditorSaveAction::Compile;
      }
      else if ( ext == ".c" || ext == ".h")
      {
        SetLanguageDefinition(TextEditor::LanguageDefinition::C());
        SetReadOnly(! has_cmake);
        action = EditorSaveAction::Compile;
      }
      else
      {
        // .cfg, .yaml, etc
        SetLanguageDefinition(TextEditor::LanguageDefinition::JeVoisCfg());
        SetReadOnly(false);
      }
    }
  }
  else
  {
    // Show the fail text:
    SetText(failtxt);
    if (failtxt.empty()) { LINFO("File " << fn << " not found -- CREATING NEW"); SetReadOnly(false); }
    else { LINFO("File " << fn << " not found."); SetReadOnly(true); }
  }

  // 如果这是一个新文件，则将一个项目添加到我们的下拉列表中：
  if (got_it == false && special_path == false)
  {
    itsItems.emplace_back(EditorItem { fn, "File " + fn.string(), action });
    itsCurrentItem = itsItems.size() - 1;
  }
  else if (fpath == "*") itsItems[itsCurrentItem].action = action; // 如果需要，则在模块的 src 上强制编译操作
  
  // 记住文件名，用于 saveFile()：
  itsFilename = fn;
  itsNewItem = itsCurrentItem;
  itsWantLoad = false;
  itsOkToLoad = false;
}

// ##############################################################################################################
std::filesystem::path const & jevois::GUIeditor::getLoadedFilePath() const
{ return itsFilename; }

// ##############################################################################################################
void jevois::GUIeditor::saveFile()
{
  LINFO("Saving " << itsFilename << " ...");
  std::ofstream os(itsFilename);
  if (os.is_open() == false) { itsHelper->reportError("Cannot write " + itsFilename.string()); return; }

  std::string const txt = GetText();
  os << txt;

  // Mark as un-edited:
  SetEdited(false);

  // 如果有 modinfo.html，则删除它，当选择信息选项卡时，GUIhelper 会重新计算它：
  std::filesystem::path mi = itsFilename.parent_path() / "modinfo.html";
  if (std::filesystem::exists(mi)) std::filesystem::remove(mi);

  // 保存后，执行任何所需的操作，如重新加载模块、重新启动、重新编译等：
  itsWantAction = true;
}

#endif // JEVOIS_PRO
