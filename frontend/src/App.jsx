import { Route, Routes } from 'react-router-dom'
import FuelPage from './pages/FuelPage.jsx'

export default function App() {
  return (
    <Routes>
      <Route path="*" element={<FuelPage />} />
    </Routes>
  )
}
