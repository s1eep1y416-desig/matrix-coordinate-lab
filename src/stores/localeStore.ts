import { create } from 'zustand';

export type Language = 'zh' | 'en';

interface LocaleState {
  language: Language;
  setLanguage: (language: Language) => void;
  toggleLanguage: () => void;
}

export const useLocaleStore = create<LocaleState>((set) => ({
  language: 'zh',
  setLanguage: (language) => set({ language }),
  toggleLanguage: () => set((state) => ({ language: state.language === 'zh' ? 'en' : 'zh' })),
}));

export const localize = (language: Language, zh: string, en: string) => language === 'zh' ? zh : en;
